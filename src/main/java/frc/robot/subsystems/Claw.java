package frc.robot.subsystems;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Claw {
  private CANSparkMax clawMotor;
  private SparkMaxPIDController clawPID;
  public Claw() 
  {
    if (Constants.clawEnabled) {
      clawMotor = new CANSparkMax(Constants.ClawConstants.motorID, MotorType.kBrushless);
    }
  }
  public void init()
  {
    
  }
  public void setCoastMode() {
    if (Constants.clawEnabled) {
      clawMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  public void setBrakeMode() {
    if (Constants.clawEnabled) {
      clawMotor.setIdleMode(IdleMode.kBrake);
    }
  }
}
