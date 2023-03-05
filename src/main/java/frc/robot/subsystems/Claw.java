package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ClawConstants.ClawMode;
import frc.utility.CanBusUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;

  private boolean stalled;
  private Timer stallTimer = new Timer();
  
  public Claw() {
    if (Constants.clawEnabled) {
      clawMotor = new CANSparkMax(Constants.ClawConstants.motorID, MotorType.kBrushless);
      CanBusUtil.staggerSparkMax(clawMotor);
    }
  }

  public void init() {
    if (Constants.clawEnabled) {
      clawMotor.restoreFactoryDefaults();
      clawMotor.setOpenLoopRampRate(Constants.ClawConstants.rampRate);
      CanBusUtil.fastVelocity(clawMotor);
      clawMotor.burnFlash();
    }
  }


  public void intake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        if (stalled) {
          clawMotor.set(ClawConstants.stallIntakePower);
        } else {
          clawMotor.set(ClawConstants.intakePower);
          DataLogManager.log("Rolly Grabbers intaking");   
        }     
      }
    }
  }

  public void outtake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        if (stalled) {
          clawMotor.set(ClawConstants.stallOuttakePower);
        } else {
          clawMotor.set(-ClawConstants.outtakePower);
          DataLogManager.log("Rolly Grabbers outtaking");   
        }  
      }
    }
  }

  public void stop() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        clawMotor.stopMotor();
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

  @Override
  public void periodic() {
    if (Constants.clawEnabled) {
      double absRPM = Math.abs(clawMotor.getEncoder().getVelocity());

      if (absRPM < ClawConstants.stallRPMLimit) {
        if (stallTimer.hasElapsed(ClawConstants.stallTime)) {
          stalled = true;
        } else {
          stallTimer.start();
        }
      } else {
        stalled = false;
        stallTimer.stop();
        stallTimer.reset();
      }
    }
  }
}
