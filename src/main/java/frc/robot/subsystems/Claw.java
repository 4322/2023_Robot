package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants.IntakeMode;
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
  public IntakeMode intakeMode;

  public Claw() {
    if (Constants.clawEnabled) {
      clawMotor = new CANSparkMax(Constants.ClawConstants.motorID, MotorType.kBrushless);
    }
  }

  public void init() {
    if (Constants.clawEnabled) {
      intakeMode = IntakeMode.stationary;
      clawMotor.restoreFactoryDefaults();
      clawMotor.setOpenLoopRampRate(Constants.ClawConstants.rampRate);
    }
  }


  public void intake() {
    if (Constants.clawEnabled) {
      clawPID.setReference(Constants.ClawConstants.IntakeVelocity,
          CANSparkMax.ControlType.kVelocity);
      intakeMode = IntakeMode.intaking;
    }
  }

  public void outtake() {
    if (Constants.clawEnabled) {
      clawPID.setReference(Constants.ClawConstants.EjectionVelocity,
          CANSparkMax.ControlType.kVelocity);
      intakeMode = IntakeMode.ejecting;
    }
  }

  public void stop() {
    if (Constants.clawEnabled) {
      clawMotor.stopMotor();
      intakeMode = IntakeMode.stationary;
    }
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
