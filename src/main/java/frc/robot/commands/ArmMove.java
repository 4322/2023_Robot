package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;

public class ArmMove extends CommandBase {

  // Use of loadSingle and loadBounce are mutually exclusive
  public enum Position {
    unknown, inBot, loadSingle, loadDouble, loadFloor, loadBounce, scoreLow, scoreMid, scoreHigh, usePreset
  }

  private static Position presetPos = Position.loadSingle;
  private static Position lastPresetScorePos = Position.scoreHigh;
  private static Position lastPos = Position.unknown;
  private static boolean safeToOuttake = false;
  private static boolean inBot = false;

  private Arm arm;
  private Telescope telescope;
  private boolean autonomous;
  private Position invokePos;
  private Position targetPos;
  private boolean armCommandedToTarget;
  private boolean telescopeCommandedToTarget;
  private boolean armAtTarget;
  private boolean telescopeAtTarget;
  private boolean done;
  //checks how long outtake has been locked for after arm has reached preset position
  private Timer telescopeAtTargetTimer = new Timer();
  private final ClawIntake clawIntake = new ClawIntake(Claw.getInstance());

  public static boolean isInBot() {
    return inBot;
  }

  public static void setArmPreset(Position pos) {
    LED.getInstance().setPresetAccepted();

    // ignore preset spamming so we don't lock-out ejecting after arm is in position
    if (pos != presetPos) {
      presetPos = pos;
      safeToOuttake = false;

      switch (pos) {
        case scoreLow:
        case scoreMid:
        case scoreHigh:
          lastPresetScorePos = pos;
          break;
        default:
          break;
      }
    }
  }

  public static void setArmPresetToLastScorePreset() {
    setArmPreset(lastPresetScorePos);
  }

  public static Position getArmPreset() {
    return ArmMove.presetPos;
  }

  public static boolean isSafeToOuttake() {
    return safeToOuttake;
  }

  public static boolean isNotReAlignPreset() {
    return ArmMove.presetPos != Position.scoreMid 
        && ArmMove.presetPos != Position.scoreHigh
        && ArmMove.presetPos != Position.loadSingle;
  }

  public ArmMove(Arm arm, Telescope telescope, Position invokePos, boolean autonomous) {
    this.arm = arm;
    this.telescope = telescope;
    this.invokePos = invokePos;
    this.autonomous = autonomous;

    addRequirements(arm, telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (invokePos == Position.usePreset) {
      targetPos = presetPos;
    } else {
      targetPos = invokePos;
    }
    armCommandedToTarget = false;
    telescopeCommandedToTarget = false;
    armAtTarget = false;
    telescopeAtTarget = false;
    done = false;
    telescopeAtTargetTimer.stop();
    telescopeAtTargetTimer.reset();
    inBot = false;

    if (presetPos == Position.scoreLow) {
      safeToOuttake = true;
    } else {
      safeToOuttake = false;
    }

    // start intake if needed
    if ((targetPos == Position.loadFloor || targetPos == Position.loadSingle) 
        && !autonomous) {
      clawIntake.schedule();
      Claw.getInstance().resetStalledIn();  // don't immediately end command if intake is already stalled
    }

    moveToTargets(true);
  }

  @Override
  public void execute() {
    if ((invokePos == Position.usePreset) && (targetPos != presetPos)
        && ((targetPos == Position.scoreMid) || (targetPos == Position.scoreHigh))
        && ((presetPos == Position.scoreMid) || (presetPos == Position.scoreHigh))) {
      // pole target changed, restart command without interrupting to keep the trigger command running
      if (!done) {
        ArmMove.lastPos = Position.unknown;
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
        case inBot:
        case loadBounce:
          telescope.moveToPosition(Constants.Telescope.inHopperPosition);
          telescopeCommandedToTarget = true;
          break;
        case loadSingle:
          if (armAtTarget) {
            telescope.moveToPosition(Constants.Telescope.loadSinglePosition);
            telescopeCommandedToTarget = true;  
          }
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
        case loadDouble:
          telescope.moveToPosition(Constants.Telescope.loadDoublePosition);
          telescopeCommandedToTarget = true;
          break;
        case scoreHigh:
          switch (ArmMove.lastPos) {
            case inBot:
            case loadSingle:
            case loadDouble:
            case loadFloor:
            case loadBounce:
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
          break;
        case loadFloor:
          switch (ArmMove.lastPos) {
            case inBot:
            case loadSingle:
            case loadBounce:
            case scoreLow:
            // exclude scoreHigh case from this optimization due to the risk of
            // impacting the floor while the telescope is extended too far
            if (armPosition >= Constants.ArmConstants.earlyTelescopeExtendPosition) {
                telescope.moveToPosition(Constants.Telescope.loadFloorPosition);
                telescopeCommandedToTarget = true;
              }
              break;
            default:
              if (armAtTarget) {
                telescope.moveToPosition(Constants.Telescope.loadFloorPosition);
                telescopeCommandedToTarget = true;
              }
              break;
          }
          break;
        default:
          break;
        
      }
      if (init && !telescopeCommandedToTarget) {
        // Positively hold telescope in so it doesn't fling out as the arm moves.
        // Needed because the telescope is stopped when the previous command is interrupted.
        telescope.moveToPosition(Constants.Telescope.inHopperPosition);            
      }
    }

    if (!armCommandedToTarget) {
      switch (targetPos) {
        case inBot:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || (((lastPos == Position.scoreHigh) || (lastPos == Position.loadFloor))
                  && (telescopePosition <= Constants.Telescope.earlyArmRetractPosition))) {
            arm.rotateToPosition(Constants.ArmConstants.inHopperPosition);
            armCommandedToTarget = true;
          }
          break;
        case loadSingle:
          if (telescopePosition <= Constants.Telescope.safeArmRetractPosition) {
            arm.rotateToPosition(Constants.ArmConstants.loadSinglePosition);
            armCommandedToTarget = true;
          }
          break;
        case loadBounce:
          if (telescopePosition <= Constants.Telescope.safeArmRetractPosition) {
            arm.rotateToPosition(Constants.ArmConstants.loadBouncePosition);
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
              || ((lastPos == Position.scoreHigh) 
                  && (telescopePosition <= Constants.Telescope.clearHighPolePosition))) {
              arm.rotateToPosition(Constants.ArmConstants.midScoringPosition);
              armCommandedToTarget = true;
          }
          break;
        case loadDouble:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || ((lastPos == Position.scoreHigh) 
                  && (telescopePosition <= Constants.Telescope.clearHighPolePosition))) {
              arm.rotateToPosition(Constants.ArmConstants.loadDoublePosition);
              armCommandedToTarget = true;
          }
          break;
        case scoreHigh:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || (lastPos == Position.scoreHigh) || (lastPos == Position.loadFloor)) {
            arm.rotateToPosition(Constants.ArmConstants.highScoringPosition);
            armCommandedToTarget = true;
          }
          break;
        case loadFloor:
          if ((telescopePosition <= Constants.Telescope.safeArmRetractPosition) 
              || (lastPos == Position.loadFloor)) {
            arm.rotateToPosition(Constants.ArmConstants.loadFloorPosition);
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
      // leave claw running so it can go into stall mode
    }
    if (!done) {
      ArmMove.lastPos = Position.unknown;
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!armAtTarget && armCommandedToTarget && arm.isAtTarget()) {
      armAtTarget = true;
      telescopeAtTargetTimer.start();
    } 
    if (!telescopeAtTarget && telescopeCommandedToTarget && telescope.isAtTarget()) {
      telescopeAtTarget = true;
    }
    if (!telescopeAtTarget && telescopeCommandedToTarget && telescopeAtTargetTimer.get() > Constants.Telescope.atTargetTimeoutSec) {
      DriverStation.reportError("Telescope move to " + targetPos.name() +  
        " timed out at position: " + telescope.getPosition(), false);
      telescopeAtTarget = true;
    }
    if (armAtTarget && telescopeAtTarget) {
      ArmMove.lastPos = targetPos;
      if ((targetPos == Position.scoreMid) || (targetPos == Position.scoreHigh)) {
        safeToOuttake = true;
      } else if (targetPos == Position.inBot) {
        inBot = true;
      }
      done = true;
      if (autonomous) {
        return true;
      }
    }

    if (!autonomous && !armAtTarget && telescopeAtTarget && armCommandedToTarget && arm.isNearTarget()
        && ((targetPos == Position.scoreMid) || (targetPos == Position.scoreHigh))) {
      safeToOuttake = true;
    }

    // end command when a game piece has been intaken
    if ((targetPos == Position.loadFloor || targetPos == Position.loadSingle)
        && Claw.getInstance().isIntakeStalled()) {
      return true;
    }

    // continue holding position in teleop until cancelled
    return false;
  }

}
