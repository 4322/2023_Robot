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

  public void changeEnum(ClawMode mode) {
    clawMode = mode;
  }


  private void intake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        if (stalledIn) {
          clawMotor.set(ClawConstants.stallIntakePower);
        } else {
          clawMotor.set(ClawConstants.intakePower);
          DataLogManager.log("Rolly Grabbers intaking");   
        }     
      }
    }
  }

  private void outtake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        if (stalledOut) {
          clawMotor.set(ClawConstants.stallOuttakePower);
        } else {
          clawMotor.set(ClawConstants.outtakePower);
          DataLogManager.log("Rolly Grabbers outtaking");   
        }  
      }
    }
  }

  private void stop() {
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
      double signedRPM = clawMotor.getEncoder().getVelocity();
      double absRPM = Math.abs(signedRPM);

      if (absRPM > ClawConstants.stallRPMLimit) {
        stalledIn = false;
        stalledOut = false;
        stallInTimer.reset();
        stallOutTimer.reset();
        stallInTimer.stop();
        stallOutTimer.stop();
      } else if ((clawMode == ClawMode.intaking) && (signedRPM < ClawConstants.stallRPMLimit)) {
        if (stallInTimer.hasElapsed(ClawConstants.stallTime)) {
          stalledIn = true;
        } else {
          stallInTimer.start();
        }
      } else if ((clawMode == ClawMode.outtaking) && (signedRPM > -ClawConstants.stallRPMLimit)) {
        if (stallOutTimer.hasElapsed(ClawConstants.stallTime)) {
          stalledOut = true;
        } else {
          stallOutTimer.start();
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
