package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class ArmMove extends CommandBase {

  public enum position {
    unknown, load, loadHigh, scoreLow, scoreMid, scoreHigh, scorePreset
  }

  private static position presetPos = position.scoreMid;
  private static position lastPos = position.unknown;
  private static boolean safeToOuttake = true;

  private Arm arm;
  private Telescope telescope;
  private boolean autonomous;
  private position invokePos;
  private position targetPos;
  private boolean armCommandedToTarget;
  private boolean telescopeCommandedToTarget;
  private boolean armAtTarget;
  private boolean telescopeAtTarget;
  private boolean done;
  private Timer timer = new Timer();
  private boolean timePrinted;

  public static void setScorePreset(position pos) {
    ArmMove.presetPos = pos;
  }

  public static boolean isSafeToOuttake() {
    return safeToOuttake;
  }

  public ArmMove(Arm arm, Telescope telescope, position invokePos, boolean autonomous) {
    this.arm = arm;
    this.telescope = telescope;
    this.invokePos = invokePos;
    this.autonomous = autonomous;

    addRequirements(arm, telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (invokePos == position.scorePreset) {
      targetPos = ArmMove.presetPos;
    } else {
      targetPos = invokePos;
    }
    armCommandedToTarget = false;
    telescopeCommandedToTarget = false;
    armAtTarget = false;
    telescopeAtTarget = false;
    done = false;
    timer.restart();
    timePrinted = false;

    if ((targetPos == position.scoreMid) || (targetPos == position.scoreHigh)) {
      safeToOuttake = false;
    } else {
      safeToOuttake = true;
    }

    moveToTargets(true);
  }

  @Override
  public void execute() {
    if ((invokePos == position.scorePreset) && (targetPos != ArmMove.presetPos)) {
      // target changed, restart command without interrupting to keep the trigger command running
      if (!done) {
        ArmMove.lastPos = position.unknown;
      }
      initialize();       
    } else {
      moveToTargets(false);
    }
  }

  private void moveToTargets(boolean init) {

    // cache values for logic consistency
    double armPosition = arm.getPosition();
    double telescopePosition = telescope.getPosition();

    if (!telescopeCommandedToTarget) {
      switch (targetPos) {
        case load:
        case loadHigh:
          telescope.moveToPosition(Constants.Telescope.loadPosition);
          telescopeCommandedToTarget = true;
          break;
        case scoreLow:
          if (armAtTarget) {
            telescope.moveToPosition(Constants.Telescope.lowScoringPosition);
            telescopeCommandedToTarget = true;  
          }
          break;
        case scoreMid:
          telescope.moveToPosition(Constants.Telescope.midScoringPosition);
          telescopeCommandedToTarget = true;
          break;
        case scoreHigh:
          switch (ArmMove.lastPos) {
            case load:
            case loadHigh:
            case scoreLow:
              if (armPosition >= Constants.ArmConstants.earlyTelescopeExtendPosition) {
                telescope.moveToPosition(Constants.Telescope.highScoringPosition);
                telescopeCommandedToTarget = true;
              }
              break;
            default:
              if (armAtTarget) {
                telescope.moveToPosition(Constants.Telescope.highScoringPosition);
                telescopeCommandedToTarget = true;
              }
              break;
          }
        default:
          break;
      }
      if (init && !telescopeCommandedToTarget) {
        // Positively hold telescope in so it doesn't fling out as the arm moves.
        // Needed because the telescope is stopped when the previous command is interrupted.
        telescope.moveToPosition(Constants.Telescope.loadPosition);            
      }
    }

    if (!armCommandedToTarget) {
      switch (targetPos) {
        case load:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || ((lastPos == position.scoreHigh) 
                  && (telescopePosition <= Constants.Telescope.earlyArmRetractPosition))) {
            arm.rotateToPosition(Constants.ArmConstants.loadPosition);
            armCommandedToTarget = true;
          }
          break;
        case loadHigh:
          if (telescopePosition <= Constants.Telescope.safeArmRetractPosition) {
            arm.rotateToPosition(Constants.ArmConstants.loadHighPosition);
            armCommandedToTarget = true;
          }
          break;
        case scoreLow:
          if (telescopePosition <= Constants.Telescope.safeArmRetractPosition) {
            arm.rotateToPosition(Constants.ArmConstants.lowScoringPosition);
            armCommandedToTarget = true;
          }
          break;
        case scoreMid:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || ((lastPos == position.scoreHigh) 
                  && (telescopePosition <= Constants.Telescope.clearHighPolePosition))) {
              arm.rotateToPosition(Constants.ArmConstants.midScoringPosition);
              armCommandedToTarget = true;
          }
          break;
        case scoreHigh:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || (lastPos == position.scoreHigh)) {
            arm.rotateToPosition(Constants.ArmConstants.highScoringPosition);
            armCommandedToTarget = true;
          }
          break;
        default:
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // let command end in autonomous mode so we can score, but hold position
    if (!autonomous) {
      arm.stop();
      telescope.stop();
    }
    if (!done) {
      ArmMove.lastPos = position.unknown;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!armAtTarget && armCommandedToTarget && arm.isAtTarget()) {
      armAtTarget = true;
    } 
    if (!telescopeAtTarget && telescopeCommandedToTarget && telescope.isAtTarget()) {
      telescopeAtTarget = true;
    }
    if (armAtTarget && telescopeAtTarget) {
      ArmMove.lastPos = targetPos;
      safeToOuttake = true;
      done = true;
      if (autonomous) {
        return true;
      } else if (Constants.debug && !timePrinted) {
        DriverStation.reportWarning("Arm move time: " + timer.get(), false);
        timePrinted = true;
      }
    }
    if (!autonomous && !armAtTarget && telescopeAtTarget && armCommandedToTarget && arm.isNearTarget()) {
      safeToOuttake = true;
    }

    // continue holding position in teleop until cancelled
    return false;
  }
}
