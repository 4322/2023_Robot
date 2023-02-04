package frc.robot.subsystems;

import java.lang.annotation.Target;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmDirection;

public class Arm extends SubsystemBase {
  private CANSparkMax motor;
  private Double currentTarget = null;
  private ArmDirection armDirection;

  public Arm() {
    if (Constants.armEnabled) {
      motor = new CANSparkMax(Constants.ArmConstants.motorID, MotorType.kBrushless);
=======
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private Double currentTarget = null;
  private Timer logTimer = new Timer();

  public Arm() {
    if (Constants.armEnabled) {
      leftMotor = new CANSparkMax(ArmConstants.motorID, MotorType.kBrushless);
      rightMotor = new CANSparkMax(ArmConstants.motorID, MotorType.kBrushless);
>>>>>>> dev
    }
  }

  public void init() {
    if (Constants.armEnabled) {
<<<<<<< HEAD
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kCoast);
      motor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
=======
      leftMotor.restoreFactoryDefaults();
      leftMotor.setIdleMode(IdleMode.kCoast);
      leftMotor.setOpenLoopRampRate(ArmConstants.rampRate);
      rightMotor.restoreFactoryDefaults();
      rightMotor.setIdleMode(IdleMode.kCoast);
      rightMotor.setOpenLoopRampRate(ArmConstants.rampRate);
      rightMotor.follow(leftMotor);
      rightMotor.setInverted(true);
      logTimer.reset();
      logTimer.start();
>>>>>>> dev
    }
  }

  public double getPosition() {
    if (Constants.armEnabled) {
      return motor.getEncoder().getPosition();
    } else {
      return -1;
    }
  }

  public boolean isAtTarget() {
    if (!Constants.armEnabled) {
      return true;
    }
    return (Math.abs(getPosition() - currentTarget) <= ArmConstants.positionTolerance);
  }

  public boolean rotateToPosition(double targetPosition) {
    if (Constants.armEnabled) {
<<<<<<< HEAD
      motor.getPIDController().setReference(targetPosition, ControlType.kPosition);
      currentTarget = targetPosition;
=======
      if ((targetPosition > ArmConstants.minPosition)
          && (targetPosition < ArmConstants.maxPosition)) {
        leftMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
        currentTarget = targetPosition;
        DataLogManager
            .log("Rotating to position " + currentTarget + " from position " + getPosition());
        return true;
      }
>>>>>>> dev
    }
    return false;
  }

  public void rotateForward() {
<<<<<<< HEAD
    motor.set(Constants.ArmConstants.forward);
    armDirection = ArmDirection.forwards;
  }

  public void rotateBackward() {
    motor.set(Constants.ArmConstants.backward);
    armDirection = ArmDirection.backwards;
  }

  public void stop() {
    motor.set(0);
    armDirection = ArmDirection.stationary;
=======
    leftMotor.set(ArmConstants.forward);
    DataLogManager.log("Arm rotating forward");
  }

  public void rotateBackward() {
    leftMotor.set(ArmConstants.backward);
    DataLogManager.log("Arm rotating backward");
>>>>>>> dev
  }

  public void setCoastMode() {
    if (Constants.armEnabled) {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.armEnabled) {
      motor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    if (Constants.armEnabled) {
      if (logTimer.hasElapsed(ArmConstants.logIntervalSeconds)) {
        DataLogManager.log("Arm position: " + getPosition());
        logTimer.reset();
      }
    }
  }
}
