package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;

public class Arm {
  private CANSparkMax armMotor1;
  private SparkMaxPIDController armPID;

  private Arm() { 
    if (Constants.armEnabled) {
        
    }
  }
}
