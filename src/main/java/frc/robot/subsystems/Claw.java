package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.utility.CanBusUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;
  private SparkMaxPIDController pidController;

  public static enum ClawMode {
    intaking,
    outtaking,
    stopped
  }

  private ClawMode clawMode = ClawMode.stopped;

  private boolean stallingIn;
  private boolean stalledIn;
  private boolean stalledOut;
  private Timer stallInTimer = new Timer();
  private Timer stallOutTimer = new Timer();
  
  private static Claw instance = null;
  public static Claw getInstance() {
    if (instance == null) {
      instance = new Claw();
    }
    return instance;
  }

  private Claw() {
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
      pidController = clawMotor.getPIDController();

      pidController.setP(ClawConstants.kP);
      pidController.setFF(ClawConstants.kF);
      pidController.setOutputRange(ClawConstants.kMinOutput, ClawConstants.kMaxOutput);

      CanBusUtil.fastVelocitySparkMax(clawMotor);
      clawMotor.burnFlash();
    }
  }

  public boolean changeState(ClawMode mode) {
    if (Constants.clawEnabled) {
      clawMode = mode;
      resetStalledIn();
      resetStalledOut();
      return true;
    } else {
      return false;
    }
  }

  private void intake() {
    resetStalledOut();
    if (stalledIn) {
      pidController.setReference(Constants.ClawConstants.stallIntakeCurrent, CANSparkMax.ControlType.kCurrent);
    } else {
      clawMotor.set(ClawConstants.intakePower);
    }   
  }

  private void outtake() {
    resetStalledIn();
    if (stalledOut) {
      pidController.setReference(Constants.ClawConstants.stallOuttakeCurrent, ControlType.kCurrent);
    } else {
      clawMotor.set(ClawConstants.outtakePower);
    }  
  }

  private void stop() {
    clawMotor.stopMotor();
    resetStalledIn();
    resetStalledOut();
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

  //detect start of game piece being received
  public boolean isIntakeStalling() {
    return stallingIn;
  }

  public boolean isIntakeStalled() {
    return stalledIn;
  }

  public void resetStalledIn() {
    stalledIn = false;
    stallingIn = false;
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
          stallingIn = true;
          if (stallInTimer.hasElapsed(ClawConstants.stallTime)) {
            stalledIn = true;
          } else if (signedRPM >= 0) { // In case we switch from intaking immediately to outtaking
            stallInTimer.start();
          }
        } else if ((clawMode == ClawMode.outtaking) && (signedRPM > -ClawConstants.stallRPMLimit)) {
          if (stallOutTimer.hasElapsed(ClawConstants.stallTime)) {
            stalledOut = true;
          } else if (signedRPM <= 0) { 
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

        LED.getInstance().setIntakeStalled(stalledIn);
      }
    }
  }
}
