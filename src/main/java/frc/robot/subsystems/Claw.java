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

  enum ClawMode {
    intaking,
    outtaking,
    stopped
  }

  private ClawMode clawMode;

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


  public void intake() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        clawMode = ClawMode.intaking;
        if (stalledIn) {
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
        clawMode = ClawMode.outtaking;
        if (stalledOut) {
          clawMotor.set(ClawConstants.stallOuttakePower);
        } else {
          clawMotor.set(ClawConstants.outtakePower);
          DataLogManager.log("Rolly Grabbers outtaking");   
        }  
      }
    }
  }

  public void stop() {
    if (Constants.clawEnabled) {
      if (!Constants.clawTuningMode) {
        clawMode = ClawMode.stopped;
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
        stallInTimer.start();
      } else if ((clawMode == ClawMode.outtaking) && (signedRPM > -ClawConstants.stallRPMLimit)) {
        stallOutTimer.start();
      }
    }
  }
}
