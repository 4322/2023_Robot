package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants.ClawMode;
import frc.utility.CanBusUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;
  public ClawMode clawMode;

  public Claw() {
    if (Constants.clawEnabled) {
      clawMotor = new CANSparkMax(Constants.ClawConstants.motorID, MotorType.kBrushless);
      CanBusUtil.staggerSparkMax(clawMotor);
    }
  }

  public void init() {
    if (Constants.clawEnabled) {
      clawMode = ClawMode.stationary;
      clawMotor.restoreFactoryDefaults();
      clawMotor.setOpenLoopRampRate(Constants.ClawConstants.rampRate);
      clawMotor.burnFlash();
    }
  }


  public void intake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        clawMotor.set(ClawConstants.IntakeVelocity);
        clawMode = ClawMode.intaking;
        DataLogManager.log("Rolly Grabbers intaking");        
      }
    }
  }

  public void outtake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        clawMotor.set(ClawConstants.EjectionVelocity);
        clawMode = ClawMode.ejecting;
        DataLogManager.log("Rolly Grabbers outtaking");
      }
    }
  }

  public void stop() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        clawMotor.stopMotor();
        clawMode = ClawMode.stationary;
        DataLogManager.log("Rolly Grabbers stopping");
      }
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
