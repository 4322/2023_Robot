package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private double scoringTarget = Constants.ArmConstants.midScoringPosition;
  private Timer logTimer = new Timer();

  private ShuffleboardTab tab;
  private GenericEntry armPos;
  private boolean homed = false;

  public Arm() {
    if (Constants.armEnabled) {
      leftMotor = new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
      rightMotor = new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);

      CanBusUtil.staggerSparkMax(leftMotor);
      CanBusUtil.staggerSparkMax(rightMotor);

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Arm");
        armPos = tab.add("Arm Position", 0).withPosition(0,0).getEntry();
      }
    }
  }

  public void init() {
    if (Constants.armEnabled) {
      leftMotor.restoreFactoryDefaults();
      leftMotor.setIdleMode(IdleMode.kBrake);
      leftMotor.setOpenLoopRampRate(ArmConstants.rampRate);
      leftMotor.setClosedLoopRampRate(ArmConstants.rampRate);
      leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.maxPosition);
      leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      rightMotor.restoreFactoryDefaults();
      rightMotor.setIdleMode(IdleMode.kBrake);
      rightMotor.setOpenLoopRampRate(ArmConstants.rampRate);
      rightMotor.follow(leftMotor, true);
      
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
      pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.positionTolerance, 0);
      CanBusUtil.dualSparkMaxPosCtrl(leftMotor, Constants.armTuningMode);

      if (Constants.armSensorEnabled) {
        armSensor = leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
      }

      leftMotor.burnFlash();
      rightMotor.burnFlash();
      logTimer.reset();
      logTimer.start();

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
      if (!Constants.armTuningMode) {
        encoder.setPosition(pos);
      }
    }
  }

  public void setScoringTarget(double target) {
    scoringTarget = target;
  }

  public double getScoringTarget() {
    return scoringTarget;
  }

  public boolean isAtTarget() {
    if (!Constants.armEnabled || currentTarget == null) {
      return true;
    }
    return (Math.abs(getPosition() - currentTarget) <= ArmConstants.positionTolerance * Constants.encoderRoundOffError);
  }

  public boolean rotateToPosition(double targetPosition) {
    if (Constants.armEnabled && homed && !Constants.armTuningMode) {
      if ((targetPosition > ArmConstants.minPosition)
          && (targetPosition < ArmConstants.maxPosition)) {
        pidController.setReference(targetPosition, ControlType.kPosition);
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
    if (Constants.armEnabled) {
      if (!Constants.armTuningMode) {
        leftMotor.stopMotor();
      }
    }
  }

  public void setHoming() {
    if (Constants.armEnabled) {
      if (!Constants.armTuningMode) {
        leftMotor.set(ArmConstants.homingPower);
      }
    }
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

  // enable/disable limit switch
  public void setLimitSwitch(boolean status) {
    if (Constants.armEnabled) {
      armSensor.enableLimitSwitch(status);
    }
  }

  public void setHomed() {
    homed = true;
  }

  public boolean isHomed() {
    return homed;
  }

  @Override
  public void periodic() {
    // separate tests to avoid dead code warnings
    if (Constants.armEnabled) {
      if (Constants.debug) {
        if (logTimer.hasElapsed(ArmConstants.logIntervalSeconds)) {
          DataLogManager.log("Arm position: " + getPosition());
          logTimer.reset();
        }
        armPos.setDouble(getPosition());
      }
    }
  }
}
