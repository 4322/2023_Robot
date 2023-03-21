package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class ArmMove extends CommandBase {
  private Arm arm;
  private Telescope telescope;
  private Double armTarget;
  private Double telescopeTarget;
  private boolean autonomous;
  private boolean armCommandedToTarget;
  private boolean telescopeCommandedToTarget;
  private boolean usePresetTargets = false;
  private Timer timer = new Timer();
  private boolean timePrinted;

  public ArmMove(Arm arm, Telescope telescope, Double armTarget, Double telescopeTarget, boolean autonomous) {
    this.arm = arm;
    this.telescope = telescope;
    this.armTarget = armTarget;
    this.telescopeTarget = telescopeTarget;
    this.autonomous = autonomous;
    if (armTarget == null) {
      usePresetTargets = true;
    }

    // interupt existing command even when presetting targets so it can be restarted with the new targets
    addRequirements(arm, telescope);
  }

  // for trigger button using previously set targets
  public ArmMove(Arm arm, Telescope telescope) {
    this(arm, telescope, null, null, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    timePrinted = false;
    if (usePresetTargets) {
      armTarget = arm.getScoringTarget();
      telescopeTarget = telescope.getScoringTarget();
    }
    armCommandedToTarget = false;
    telescopeCommandedToTarget = false;
    moveToTargets(true);
  }

  @Override
  public void execute() {
    if (usePresetTargets && ((armTarget != arm.getScoringTarget()) || 
        telescopeTarget != telescope.getScoringTarget())) {
      // target(s) changed, restart command without interrupting to keep the trigger command running
      armTarget = arm.getScoringTarget();
      telescopeTarget = telescope.getScoringTarget();   
      initialize();       
    } else {
      moveToTargets(false);
    }
  }

  private void moveToTargets(boolean init) {
    if (!telescopeCommandedToTarget) {
      if ((telescopeTarget <= Constants.Telescope.safePosition) || 
          (arm.getPosition() >= ArmConstants.telescopeExtendablePosition)) {
        telescope.moveToPosition(telescopeTarget);
        telescopeCommandedToTarget = true;
      } else if (init) {
        // positively hold telescope in so it doesn't fling out as the arm moves up
        telescope.moveToPosition(Constants.Telescope.loadPosition);
      }
    }
    if (!armCommandedToTarget) {
      if ((armTarget >= ArmConstants.telescopeExtendablePosition) || 
          (telescope.getPosition() <= Constants.Telescope.safePosition)) {
            arm.rotateToPosition(armTarget);
            armCommandedToTarget = true;
      } else if (init) {
        // rotate arm back only to the safe point
        //arm.rotateToPosition(ArmConstants.telescopeExtendablePosition);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armCommandedToTarget && telescopeCommandedToTarget && arm.isAtTarget() && telescope.isAtTarget()) {
      if (autonomous) {
        return true;
      } else if (Constants.debug && !timePrinted) {
        DriverStation.reportError("Arm move time: " + timer.get(), false);
        timePrinted = true;
      }
    }

    // continue holding position until cancelled in teleop
    return false;
  }
}
