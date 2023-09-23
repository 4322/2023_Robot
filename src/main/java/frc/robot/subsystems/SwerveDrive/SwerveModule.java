package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.revrobotics.CANSparkMax;
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
import edu.wpi.first.math.MathUtil;

public class SwerveModule extends ControlModule {
  private CANSparkMax turningMotor;
  private TalonFX driveMotor;
  private TalonFX driveMotor2;
  private SparkMaxAbsoluteEncoder encoder;
  private WheelPosition wheelPosition;
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
    configDrive(driveMotor, wheelPosition);
    configDrive(driveMotor2, wheelPosition);
    driveMotor2.setControl(new Follower(driveMotor.getDeviceID(), false));
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
    
    talon.getConfigurator().apply(slot0config);
    talon.getConfigurator().apply(cLoopRampsConfigs);
    talon.getConfigurator().apply(mOutputConfigs);
    
    // Invert the left side modules so we can zero all modules with the bevel gears facing inward.
    // Without this code, all bevel gears would need to face left when the modules are zeroed.
    boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
    talon.setInverted(isLeftSide);

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
    config.setOutputRange(-DriveConstants.Rotation.maxPower, DriveConstants.Rotation.maxPower);
    sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling

    sparkMax.enableVoltageCompensation(DriveConstants.Rotation.configVoltageCompSaturation); 
    sparkMax.setSmartCurrentLimit(DriveConstants.Rotation.stallLimit, DriveConstants.Rotation.freeLimit); 
    encoder.setPositionConversionFactor(360);  // convert encoder position duty cycle to degrees
    sparkMax.getPIDController().setFeedbackDevice(encoder);
    sparkMax.getPIDController().setPositionPIDWrappingEnabled(true);
    sparkMax.getPIDController().setPositionPIDWrappingMinInput(0);
    sparkMax.getPIDController().setPositionPIDWrappingMaxInput(360);

    // need rapid position feedback for steering control
    CanBusUtil.fastPositionSparkMax(turningMotor);
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
    if (Constants.driveEnabled) {
      if (!Constants.driveTuningMode) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(encoder.getPosition()));

        driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond
                / (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
                * DriveConstants.Drive.gearRatio * DriveConstants.encoderResolution / 10));
        turningMotor.getPIDController().setReference(
          MathUtil.inputModulus(state.angle.getDegrees(), 0, 360), 
          ControlType.kPosition);
      }
    }
  }

  public void setCoastmode() {
    if (Constants.driveEnabled) {
      driveMotor.setControl(new CoastOut());
      turningMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      driveMotor.setControl(new StaticBrake());
      turningMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      if (!Constants.driveTuningMode) {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
      }
    }
  }
}
