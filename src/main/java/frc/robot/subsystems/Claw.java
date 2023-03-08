package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.utility.CanBusUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;

  public static enum ClawMode {
    intaking,
    outtaking,
    stopped
  }

  private ClawMode clawMode = ClawMode.stopped;

  private boolean stalledIn;
  private boolean stalledOut;
  private Timer stallInTimer = new Timer();
  private Timer stallOutTimer = new Timer();
  
  public Claw() {
    if (Constants.clawEnabled) {
      clawMotor = new CANSparkMax(Constants.ClawConstants.motorID, MotorType.kBrushless);
      CanBusUtil.staggerSparkMax(clawMotor);
      stallInTimer.reset();
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

  public void changeState(ClawMode mode) {
    clawMode = mode;
    if (clawMode == ClawMode.intaking) {
      DataLogManager.log("Claw intaking");
    }
    if (clawMode == ClawMode.outtaking) {
      DataLogManager.log("Claw outtaking");
    }
    if (clawMode == ClawMode.stopped) {
      DataLogManager.log("Claw stopped");
    }
  }

  private void intake() {
    resetStalledOut();
    if (stalledIn) {
      clawMotor.set(ClawConstants.stallIntakePower);
    } else {
      clawMotor.set(ClawConstants.intakePower);
    }     
  }

  private void outtake() {
    resetStalledIn();
    if (stalledOut) {
      clawMotor.set(ClawConstants.stallOuttakePower);
    } else {
      clawMotor.set(ClawConstants.outtakePower);
    }  
  }

  private void stop() {
    clawMotor.stopMotor();
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

  private void resetStalledIn() {
    stalledIn = false;
    stallInTimer.reset();
    stallInTimer.stop();
  }
  
  private void resetStalledOut() {
    stalledOut = false;
    stallOutTimer.reset();
    stallOutTimer.stop();
  }

  @Override
  public void periodic() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        double signedRPM = clawMotor.getEncoder().getVelocity();
        double absRPM = Math.abs(signedRPM);

        if (absRPM > ClawConstants.stallRPMLimit) {
          resetStalledIn();
          resetStalledOut();
        } else if ((clawMode == ClawMode.intaking) && (signedRPM < ClawConstants.stallRPMLimit)) {
          if (stallInTimer.hasElapsed(ClawConstants.stallTime)) {
            stalledIn = true;
          } else {
            if (signedRPM >= 0) {
              stallInTimer.start();
            }
          }
        } else if ((clawMode == ClawMode.outtaking) && (signedRPM < ClawConstants.stallRPMLimit)) {
          if (stallOutTimer.hasElapsed(ClawConstants.stallTime)) {
            stalledOut = true;
          } else {
            if (signedRPM <= 0) {
              stallOutTimer.start();
            }
          }
        }

        if (clawMode == ClawMode.intaking) {
          intake();
        } else if (clawMode == ClawMode.outtaking) {
          outtake();
        } else {
          stop();
        }
      }
    }
  }
}
