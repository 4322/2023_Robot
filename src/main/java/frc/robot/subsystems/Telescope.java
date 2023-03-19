package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import frc.utility.CanBusUtil;

public class Telescope extends SubsystemBase {
  private CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private Double currentTarget = null;
  private Timer logTimer = new Timer();

  private ShuffleboardTab tab;
  private GenericEntry posTab;
  private boolean homed = false;

  public Telescope() {
    if (Constants.telescopeEnabled) {
      motor = new CANSparkMax(Constants.Telescope.motorID, MotorType.kBrushless);
      CanBusUtil.staggerSparkMax(motor);

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Telescope");
        posTab = tab.add("Telescope Position", 0).withPosition(0,0).getEntry();
      }
    }
  }

  public void init() {
    if (Constants.telescopeEnabled) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kBrake);
      motor.setOpenLoopRampRate(Constants.Telescope.rampRate);
      motor.setClosedLoopRampRate(Constants.Telescope.rampRate);
      motor.setSoftLimit(SoftLimitDirection.kForward, Constants.Telescope.maxPosition);
      motor.enableSoftLimit(SoftLimitDirection.kForward, true);
      
      encoder = motor.getEncoder();
      pidController = motor.getPIDController();

      pidController.setP(Constants.Telescope.SmartMotion.kP);
      pidController.setI(Constants.Telescope.SmartMotion.kI);
      pidController.setD(Constants.Telescope.SmartMotion.kD);
      pidController.setIZone(Constants.Telescope.SmartMotion.kIz);
      pidController.setOutputRange(Constants.Telescope.SmartMotion.kMinOutput, Constants.Telescope.SmartMotion.kMaxOutput);
      pidController.setSmartMotionMinOutputVelocity(Constants.Telescope.SmartMotion.minVel, 0);
      pidController.setSmartMotionMaxVelocity(Constants.Telescope.SmartMotion.maxVel, 0);
      pidController.setSmartMotionMaxAccel(Constants.Telescope.SmartMotion.maxAcc, 0);
      pidController.setSmartMotionAllowedClosedLoopError(Constants.Telescope.positionTolerance, 0);
      CanBusUtil.fastPositionSparkMax(motor);

      motor.burnFlash();
      logTimer.reset();
      logTimer.start();
    }
  }

  public double getPosition() {
    if (Constants.telescopeEnabled) {
      return encoder.getPosition();
    } else {
      return -1;
    }
  }

  public void setPosition(double pos) {
    if (Constants.telescopeEnabled) {
      if (!Constants.telescopeTuningMode) {
        encoder.setPosition(pos);
      }
    }
  }

  public boolean isAtTarget() {
    if (!Constants.telescopeEnabled) {
      return true;
    }
    return (Math.abs(getPosition() - currentTarget) <= Constants.Telescope.positionTolerance);
  }

  public boolean moveToPosition(double targetPosition) {
    if (Constants.telescopeEnabled && homed && !Constants.telescopeTuningMode) {
      if ((targetPosition > Constants.Telescope.minPosition)
          && (targetPosition < Constants.Telescope.maxPosition)) {
        pidController.setReference(targetPosition, ControlType.kPosition);
        currentTarget = targetPosition;
        DataLogManager
            .log("Telescoping to position " + currentTarget + " from position " + getPosition());
        return true;
      }
    }
    return false;
  }

  public void stop() {
    if (Constants.telescopeEnabled) {
      if (!Constants.telescopeTuningMode) {
        motor.stopMotor();
      }
    }
  }

  public void setHoming() {
    if (Constants.telescopeEnabled) {
      if (!Constants.telescopeTuningMode) {
        motor.set(Constants.Telescope.homingPower);
      }
    }
  }

  public void setCoastMode() {
    if (Constants.telescopeEnabled) {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.telescopeEnabled) {
      motor.setIdleMode(IdleMode.kBrake);
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
    if (Constants.telescopeEnabled) {
      if (Constants.debug) {
        posTab.setDouble(getPosition());
      }
    }
  }
}
