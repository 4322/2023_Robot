package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utility.CanBusUtil;

public class Arm extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private SparkMaxLimitSwitch armSensor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private Double currentTarget = null;
  private Timer logTimer = new Timer();

  public Arm() {
    if (Constants.armEnabled) {
      leftMotor = new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
      rightMotor = new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);

      CanBusUtil.staggerSparkMax(leftMotor);
      CanBusUtil.staggerSparkMax(rightMotor);
    }
  }

  public void init() {
    if (Constants.armEnabled) {
      leftMotor.restoreFactoryDefaults();
      leftMotor.setIdleMode(IdleMode.kBrake);
      //leftMotor.setOpenLoopRampRate(ArmConstants.rampRate);
      rightMotor.restoreFactoryDefaults();
      rightMotor.setIdleMode(IdleMode.kBrake);
      //rightMotor.setOpenLoopRampRate(ArmConstants.rampRate);
      rightMotor.follow(leftMotor, true);
      CanBusUtil.dualSparkMaxPosCtrl(leftMotor);
      logTimer.reset();
      logTimer.start();

      encoder = leftMotor.getEncoder();
      pidController = leftMotor.getPIDController();

      pidController.setP(ArmConstants.SmartMotion.kP);
      pidController.setI(ArmConstants.SmartMotion.kI);
      pidController.setD(ArmConstants.SmartMotion.kD);
      pidController.setIZone(ArmConstants.SmartMotion.kIz);
      pidController.setOutputRange(ArmConstants.SmartMotion.kMinOutput, ArmConstants.SmartMotion.kMaxOutput);
      pidController.setSmartMotionMinOutputVelocity(ArmConstants.SmartMotion.minVel, 0);
      pidController.setSmartMotionMaxVelocity(ArmConstants.SmartMotion.maxVel, 0);
      pidController.setSmartMotionMaxAccel(ArmConstants.SmartMotion.maxAcc, 0);
      pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.SmartMotion.positionTolerance, 0);

      if (Constants.armSensorEnabled) {
        armSensor = leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      }
    }
  }

  public double getPosition() {
    if (Constants.armEnabled) {
      return encoder.getPosition();
    } else {
      return -1;
    }
  }

  public void setPosition(double pos) {
    if (Constants.armEnabled) {
      leftMotor.getEncoder().setPosition(pos);
    }
  }
  public boolean isAtTarget() {
    if (!Constants.armEnabled) {
      return true;
    }
    return (Math.abs(getPosition() - currentTarget) <= ArmConstants.SmartMotion.positionTolerance);
  }

  public boolean rotateToPosition(double targetPosition) {
    if (Constants.armEnabled) {
      if ((targetPosition > ArmConstants.minPosition)
          && (targetPosition < ArmConstants.maxPosition)) {
        pidController.setReference(targetPosition, ControlType.kSmartMotion);
        currentTarget = targetPosition;
        DataLogManager
            .log("Rotating to position " + currentTarget + " from position " + getPosition());
        return true;
      }
    }
    return false;
  }

  public boolean getArmSensorPressed() {
    if (Constants.armSensorEnabled) {
      return armSensor.isPressed();
    } else {
      return false;
    }
  }

  public void stop() {
    leftMotor.stopMotor();
  }
  //rotate methods not being used right now
  public void rotateForward() {
    leftMotor.set(ArmConstants.forward);
    DataLogManager.log("Arm rotating forward");
  }

  public void rotateBackward() {
    leftMotor.set(ArmConstants.backward);
    DataLogManager.log("Arm rotating backward");
  }

  public void setArmSpeed(double speed) {
    leftMotor.set(speed);
  }

  public void setCoastMode() {
    if (Constants.armEnabled) {
      leftMotor.setIdleMode(IdleMode.kCoast);
      rightMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.armEnabled) {
      leftMotor.setIdleMode(IdleMode.kBrake);
      rightMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    if (Constants.armEnabled) {
      if (logTimer.hasElapsed(ArmConstants.logIntervalSeconds) && Constants.debug) {
        DataLogManager.log("Arm position: " + getPosition());
        logTimer.reset();
      }
    }
  }
}
