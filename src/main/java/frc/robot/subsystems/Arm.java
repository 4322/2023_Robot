package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    }
  }

  public void init() {
    if (Constants.armEnabled) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kCoast);
      motor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
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
    return (Math.abs(getPosition() - currentTarget) <= Constants.ArmConstants.positionTolerance);
  }

  public boolean rotateToPosition(double targetPosition) {
    if (Constants.armEnabled) {
      motor.getPIDController().setReference(targetPosition, ControlType.kPosition);
      currentTarget = targetPosition;
    }
    return false;
  }

  public void rotateForward() {
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
}
