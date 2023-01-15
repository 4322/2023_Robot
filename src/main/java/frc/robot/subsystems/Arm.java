package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Arm {
  private CANSparkMax leftMotor;

  public Arm() {
    if (Constants.armEnabled) {
      leftMotor = new CANSparkMax(Constants.ArmConstants.motorID, MotorType.kBrushless);
    }
  }

  public void init() {
    if (Constants.armEnabled) {
      leftMotor.restoreFactoryDefaults();
      leftMotor.setIdleMode(IdleMode.kCoast);
      leftMotor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
    }
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
    }
  }

  public void setBrakeMode() {
    if (Constants.armEnabled) {
      leftMotor.setIdleMode(IdleMode.kBrake);
    }
  }
}
