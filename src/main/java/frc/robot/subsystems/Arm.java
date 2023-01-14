package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Arm {
  private CANSparkMax leftMotor;
  private SparkMaxPIDController armPID;

  private enum rotationDir {
    forward,
    backward,
    none
}

  private Arm() { 
    if (Constants.armEnabled) {
        leftMotor = new CANSparkMax(Constants.ArmConstants.motorID, MotorType.kBrushless);
    }
    
    public void init() {
        if (Constants.armEnabled) {
            leftMotor.restoreFactoryDefaults();
            leftMotor.setIdleMode(IdleMode.kCoast);
            leftMotor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
            armEncoder = armEncoder.getEncoder();
        }
    }

    

    public void rotateToPos(double targetPos) {

    }

    public void rotate() {
        if (Constants.armEnabled) {
            
        }
    }

    public void setCurrentPosition(double pos) {
        leftMotor.set
    }

    public void setCoastMode() {
        if (Constants.armEnabled) {
            leftMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setBreakMode() {
        if (Constants.armEnabled) {
            leftMotor.setIdleMode(IdleMode.kBrake);
        }
    }
  }
}
