package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Telescope;

public class ArmMove extends CommandBase {

  // Use of loadSingle and loadBounce are mutually exclusive
  public enum Position {
    unknown, inHopper, loadSingle, loadDouble, loadFloor, loadBounce, scoreLow, scoreMid, scoreHigh, usePreset
  }

  private static Position presetPos = Position.scoreHigh;
  private static Position lastPresetScorePos = Position.scoreHigh;
  private static Position lastPos = Position.unknown;
  private static boolean safeToOuttake = true;

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
  private Timer timer = new Timer();
  private boolean timePrinted;
  private final ClawIntake clawIntake = new ClawIntake(Claw.getInstance());

  public static void setArmPreset(Position pos) {
    ArmMove.presetPos = pos;

    switch (pos) {
      case scoreLow:
      case scoreMid:
      case scoreHigh:
        lastPresetScorePos = pos;
        break;
      default:
        break;
    }
    
    // update trigger lock in case driver pulls the right trigger before the left trigger
    if ((pos == Position.scoreMid) || (pos == Position.scoreHigh)) {
      safeToOuttake = false;
    } else {
      safeToOuttake = true;
    }
  }

  public static Position getArmPreset() {
    return ArmMove.presetPos;
  }

  public static boolean isSafeToOuttake() {
    return safeToOuttake;
  }

  public static boolean isNotForwardScoringPreset() {
    return ArmMove.presetPos != Position.scoreMid && ArmMove.presetPos != Position.scoreHigh;
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

    if ((targetPos == Position.scoreMid) || (targetPos == Position.scoreHigh)) {
      safeToOuttake = false;
    } else {
      safeToOuttake = true;
    }

    // start intake if needed
    if ((targetPos == Position.loadFloor || targetPos == Position.loadSingle) 
        && !autonomous) {
      clawIntake.schedule();
    }

    moveToTargets(true);
  }

  @Override
  public void execute() {
    if ((invokePos == Position.usePreset) && (targetPos != ArmMove.presetPos)) {
      // target changed, restart command without interrupting to keep the trigger command running
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
        case inHopper:
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
            case inHopper:
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
            case inHopper:
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
        case inHopper:
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
      if (clawIntake.isScheduled() && !Claw.getInstance().isIntakeStalled()) {
        clawIntake.cancel();
      }
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
      } else if (!timePrinted) {
        DataLogManager.log("Arm move time: " + timer.get());
        timePrinted = true;
      }
    }
    if (!autonomous && !armAtTarget && telescopeAtTarget && armCommandedToTarget && arm.isNearTarget()) {
      safeToOuttake = true;
    }

    // end command when a game piece has been intaked
    if ((targetPos == Position.loadFloor || targetPos == Position.loadSingle)
        && Claw.getInstance().isIntakeStalled()) {
      presetPos = lastPresetScorePos;
      return true;
    }

    // continue holding position in teleop until cancelled
    return false;
  }

}
