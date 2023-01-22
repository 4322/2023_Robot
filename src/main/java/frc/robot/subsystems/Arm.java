package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Arm {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private Double currentTarget = null;

  public Arm() {
    if (Constants.armEnabled) {
      leftMotor = new CANSparkMax(Constants.ArmConstants.motorID, MotorType.kBrushless);
      rightMotor = new CANSparkMax(Constants.ArmConstants.motorID, MotorType.kBrushless);
    }
  }

  public void init() {
    if (Constants.armEnabled) {
      leftMotor.restoreFactoryDefaults();
      leftMotor.setIdleMode(IdleMode.kCoast);
      leftMotor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
      rightMotor.restoreFactoryDefaults();
      rightMotor.setIdleMode(IdleMode.kCoast);
      rightMotor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
      rightMotor.follow(leftMotor);
      rightMotor.setInverted(true);
    }
  }

  public double getPosition() {
    if (Constants.armEnabled) {
      return leftMotor.getEncoder().getPosition();
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
      leftMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
      currentTarget = targetPosition;
    }
    return false;
  }

  public void rotateForward() {
    leftMotor.set(Constants.ArmConstants.forward);
  }

  public void rotateBackward() {
    leftMotor.set(Constants.ArmConstants.backward);
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
}
