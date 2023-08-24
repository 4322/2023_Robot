package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private WPI_TalonFX driveMotor;
  private WPI_TalonFX driveMotor2;
  private SparkMaxAbsoluteEncoder encoder;
  private WheelPosition wheelPosition;

  public SwerveModule(int rotationID, int wheelID, int wheelID2, WheelPosition pos, int encoderID) {
    super(pos);
    turningMotor = new CANSparkMax(rotationID, MotorType.kBrushless);
    driveMotor = new WPI_TalonFX(wheelID);
    driveMotor2 = new WPI_TalonFX(wheelID2);
    encoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    wheelPosition = pos;

    CanBusUtil.staggerTalonStatusFrames(driveMotor);
    CanBusUtil.staggerTalonStatusFrames(driveMotor2);
    CanBusUtil.staggerSparkMax(turningMotor);
  }

  public void init() {
    driveMotor2.follow(driveMotor,FollowerType.PercentOutput);
    driveMotor2.setInverted(TalonFXInvertType.OpposeMaster);
    configDrive(driveMotor, wheelPosition);
    configDrive(driveMotor2, wheelPosition);
    configRotation(turningMotor);
  }

  private void configDrive(WPI_TalonFX talon, WheelPosition pos) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = DriveConstants.Drive.kP;
    config.slot0.kI = DriveConstants.Drive.kI;
    config.slot0.kD = DriveConstants.Drive.kD;
    config.slot0.integralZone = DriveConstants.Drive.kIz;
    config.slot0.kF = DriveConstants.Drive.kFF;
    config.closedloopRamp = DriveConstants.Drive.configClosedLoopRamp;
    config.neutralDeadband = DriveConstants.Drive.brakeModeDeadband; // delay brake mode activation
                                                                     // for tipping

    talon.configAllSettings(config);
    
    talon.setNeutralMode(NeutralMode.Coast); // Allow robot to be moved prior to enabling
    boolean isRightSide = pos == WheelPosition.FRONT_RIGHT || pos == WheelPosition.BACK_RIGHT;
    talon.setInverted(!isRightSide);
    talon.setSensorPhase(false);

    talon.configVoltageCompSaturation(DriveConstants.Drive.voltageCompSaturation);
    talon.enableVoltageCompensation(DriveConstants.Drive.enableVoltageCompensation);

    talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
        DriveConstants.Drive.statorEnabled, DriveConstants.Drive.statorLimit,
        DriveConstants.Drive.statorThreshold, DriveConstants.Drive.statorTime));
    talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
        DriveConstants.Drive.supplyEnabled, DriveConstants.Drive.supplyLimit,
        DriveConstants.Drive.supplyThreshold, DriveConstants.Drive.supplyTime));

    // need rapid velocity feedback for anti-tipping logic

    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
        CanBusUtil.nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);

  }
 
  private void configRotation(CANSparkMax sparkMax) {
    SparkMaxPIDController config = sparkMax.getPIDController();
    config.setP(DriveConstants.Rotation.kP,0);
    config.setD(DriveConstants.Rotation.kD,0);
    sparkMax.setClosedLoopRampRate(DriveConstants.Rotation.configCLosedLoopRamp);
    config.setSmartMotionAllowedClosedLoopError(DriveConstants.Rotation.allowableClosedloopError,0);
    config.setOutputRange(-DriveConstants.Rotation.minPower, DriveConstants.Rotation.maxPower);
    sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling
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
    return OrangeMath.falconEncoderToMeters(driveMotor.getSelectedSensorPosition(0),
        OrangeMath.getCircumference(OrangeMath.inchesToMeters(DriveConstants.Drive.wheelDiameterInches)),
        DriveConstants.Drive.gearRatio);
  }

  @Override
  public double getVelocity() {
    // feet per second
    return driveMotor.getSelectedSensorVelocity(0) * 10 / DriveConstants.encoderResolution
        / Constants.DriveConstants.Drive.gearRatio * Math.PI
        * Constants.DriveConstants.Drive.wheelDiameterInches / 12;
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

    driveMotor.set(ControlMode.Velocity,
        state.speedMetersPerSecond
            / (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
            * DriveConstants.Drive.gearRatio * DriveConstants.encoderResolution / 10); // every 100
                                                                                       // ms

    // Calculate the change in degrees and add that to the current position
    turningMotor.getPIDController().setReference(currentDeg + OrangeMath.boundDegrees(state.angle.getDegrees() - currentDeg), 
      ControlType.kPosition);

  }

  public void setCoastmode() {
    driveMotor.setNeutralMode(NeutralMode.Coast);
    turningMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setIdleMode(IdleMode.kBrake);
  }

  public void stop() {
    driveMotor.stopMotor();
    turningMotor.stopMotor();
  }
}
