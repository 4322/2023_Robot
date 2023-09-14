package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utility.OrangeMath;
import frc.utility.CanBusUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveModule extends ControlModule {
  private CANSparkMax turningMotor;
  private TalonFX driveMotor;
  private TalonFX driveMotor2;
  private SparkMaxAbsoluteEncoder encoder;
  private WheelPosition wheelPosition;
  private VoltageOut voltageRequest = new VoltageOut(0);
  private CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();

  public SwerveModule(int rotationID, int wheelID, int wheelID2, WheelPosition pos, int encoderID) {
    super(pos);
    turningMotor = new CANSparkMax(rotationID, MotorType.kBrushless);
    driveMotor = new TalonFX(wheelID, Constants.DriveConstants.Drive.canivoreName);
    driveMotor2 = new TalonFX(wheelID2, Constants.DriveConstants.Drive.canivoreName);
    encoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    wheelPosition = pos;

    CanBusUtil.staggerSparkMax(turningMotor);
  }

  public void init() {
    driveMotor2.setControl(new Follower(driveMotor.getDeviceID(), true));
    configDrive(driveMotor, wheelPosition);
    configDrive(driveMotor2, wheelPosition);
    configRotation(turningMotor);
  }

  private void configDrive(TalonFX talon, WheelPosition pos) {
    Slot0Configs slot0config = new Slot0Configs();
    slot0config.kP = DriveConstants.Drive.kP;
    slot0config.kI = DriveConstants.Drive.kI;
    slot0config.kD = DriveConstants.Drive.kD;
    slot0config.kV = DriveConstants.Drive.kV;

    MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
    ClosedLoopRampsConfigs cLoopRampsConfigs = new ClosedLoopRampsConfigs();

    mOutputConfigs.NeutralMode = NeutralModeValue.Coast; // Allow robot to be moved prior to enabling
    mOutputConfigs.DutyCycleNeutralDeadband = DriveConstants.Drive.brakeModeDeadband;  // delay brake mode activation
                                                                                       // for tipping
    cLoopRampsConfigs.VoltageClosedLoopRampPeriod = DriveConstants.Drive.configClosedLoopRamp;
    //slot0config.closedloopRamp = DriveConstants.Drive.configClosedLoopRamp;
    
    talon.getConfigurator().apply(slot0config);
    talon.getConfigurator().apply(cLoopRampsConfigs);
    talon.getConfigurator().apply(mOutputConfigs);
    
    boolean isRightSide = pos == WheelPosition.FRONT_RIGHT || pos == WheelPosition.BACK_RIGHT;
    talon.setInverted(!isRightSide);

    talon.setControl(voltageRequest.withOutput(DriveConstants.Drive.voltageCompSaturation));

    // applies stator & supply current limit configs to device
    // refer to https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html 
    currentLimitConfigs.StatorCurrentLimitEnable = DriveConstants.Drive.statorEnabled;
    currentLimitConfigs.StatorCurrentLimit = DriveConstants.Drive.statorLimit;
    currentLimitConfigs.SupplyCurrentLimit = DriveConstants.Drive.supplyLimit;
    currentLimitConfigs.SupplyCurrentThreshold = DriveConstants.Drive.supplyThreshold;
    currentLimitConfigs.SupplyTimeThreshold = DriveConstants.Drive.supplyTime;
    currentLimitConfigs.SupplyCurrentLimitEnable = DriveConstants.Drive.supplyEnabled;
    talon.getConfigurator().apply(currentLimitConfigs);
    
    // need rapid velocity feedback for anti-tipping logic
    talon.getPosition().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
      Constants.controllerConfigTimeoutMs);
  }
 
  private void configRotation(CANSparkMax sparkMax) {
    SparkMaxPIDController config = sparkMax.getPIDController();
    config.setP(DriveConstants.Rotation.kP,0);
    config.setD(DriveConstants.Rotation.kD,0);
    sparkMax.setClosedLoopRampRate(DriveConstants.Rotation.configCLosedLoopRamp);
    config.setSmartMotionAllowedClosedLoopError(DriveConstants.Rotation.allowableClosedloopError,0);
    config.setOutputRange(-DriveConstants.Rotation.minPower, DriveConstants.Rotation.maxPower); //min and max power need to be adjusted
    sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling

    //voltage control
    sparkMax.enableVoltageCompensation(DriveConstants.Rotation.configVoltageCompSaturation); 
    sparkMax.setSmartCurrentLimit(DriveConstants.Rotation.stallLimit, DriveConstants.Rotation.freeLimit); 

    // THE ENCODER WILL GIVE YOU DEGREES
    encoder.setPositionConversionFactor(360);

    try {
      Thread.sleep(50); // 5 status frames to be safe
    } catch (InterruptedException e) {
    }

    REVLibError error =
        encoder.setZeroOffset(DriveConstants.Rotation.CANCoderOffsetDegrees[position.wheelNumber]);
    if (error != REVLibError.kOk) {
      DriverStation.reportError(
          "Error " + error.value + " initializing sparkMax " + sparkMax.getDeviceId() + " position ",
          false); //FIX
    }

    // need rapid position feedback for steering logic
    CanBusUtil.fastPositionSparkMax(turningMotor);
  } 

  public double getInternalRotationCount() {
    return encoder.getPosition();
  }

  public double getInternalRotationDegrees() {
    return OrangeMath.boundDegrees(encoder.getPosition());
  }

  @Override
  public double getDistance() {
    return OrangeMath.falconRotationsToMeters(driveMotor.getRotorPosition().getValue(),
        OrangeMath.getCircumference(OrangeMath.inchesToMeters(DriveConstants.Drive.wheelDiameterInches)),
        DriveConstants.Drive.gearRatio);
  }

  @Override
  public double getVelocity() {
    // feet per second
    return driveMotor.getRotorVelocity().getValue() * 10 / Constants.DriveConstants.Drive.gearRatio 
        * Math.PI * Constants.DriveConstants.Drive.wheelDiameterInches / 12;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity() * Constants.feetToMeters, 
      Rotation2d.fromDegrees(encoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(encoder.getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    double currentDeg = encoder.getPosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(currentDeg));

    driveMotor.set(state.speedMetersPerSecond
            / (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
            * DriveConstants.Drive.gearRatio * DriveConstants.encoderResolution / 10); // every 100ms

    // Calculate the change in degrees and add that to the current position
    turningMotor.getPIDController().setReference(currentDeg + OrangeMath.boundDegrees(state.angle.getDegrees() - currentDeg), 
      ControlType.kPosition);
  }

  public void setCoastmode() {
    driveMotor.setControl(new CoastOut());
    turningMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    driveMotor.setControl(new StaticBrake());
    turningMotor.setIdleMode(IdleMode.kBrake);
  }

  public void stop() {
    driveMotor.stopMotor();
    turningMotor.stopMotor();
  }
}
