package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;
  private SparkMaxPIDController clawPID;

  public Claw() {
    if (Constants.clawEnabled) {
      clawMotor = new CANSparkMax(Constants.ClawConstants.motorID, MotorType.kBrushed);
    }
  }

  public void init() {
    if (Constants.clawEnabled) {
      clawMotor.restoreFactoryDefaults();
      clawMotor.setOpenLoopRampRate(Constants.ClawConstants.rampRate);
    }
  }

  public void manualIntake() {
    clawMotor.set(ClawConstants.clawSpeed);
  }

  public void manualOuttake() {
    clawMotor.set(-ClawConstants.clawSpeed);
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

  public void manualStop() {
    if (Constants.clawEnabled) {
      clawMotor.stopMotor();
    }
  }
}
