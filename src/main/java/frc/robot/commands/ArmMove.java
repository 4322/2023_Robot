package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class ArmMove extends CommandBase {
  private Arm arm;
  private Telescope telescope;
  private static double presetArmTarget;
  private static double presetTelescopeTarget;
  private double armTarget;
  private double telescopeTarget;
  private boolean presetTargets;
  private boolean autonomous;
  private boolean armCommandedToTarget;
  private boolean telescopeCommandedToTarget;

  // all parameters
  public ArmMove(Arm arm, Telescope telescope, double armTarget, double telescopeTarget, boolean autonomous, boolean presetTargets) {
    this.arm = arm;
    this.telescope = telescope;
    this.armTarget = armTarget;
    this.telescopeTarget = telescopeTarget;
    this.autonomous = autonomous;
    this.presetTargets = presetTargets;

    // interupt existing command even when presetting targets so it can be restarted with the new targets
    addRequirements(arm, telescope);
  }

  // for trigger button using previously set targets
  public ArmMove(Arm arm, Telescope telescope) {
    this(arm, telescope, ArmMove.presetArmTarget, ArmMove.presetTelescopeTarget, false, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armCommandedToTarget = false;
    telescopeCommandedToTarget = false;

    if (presetTargets) {
      ArmMove.presetArmTarget = armTarget;
      ArmMove.presetTelescopeTarget = telescopeTarget;
    } else {
      moveToTargets(true);
    }
  }

  @Override
  public void execute() {
    if (!presetTargets) {
      moveToTargets(false);
    }
  }

  private void moveToTargets(boolean init) {
    if (!telescopeCommandedToTarget) {
      if ((telescopeTarget <= Constants.Telescope.safePosition) || 
          (arm.getPosition() >= ArmConstants.telescopeExtendablePosition)) {
        telescope.moveToPosition(telescopeTarget);
        telescopeCommandedToTarget = true;
      } 
    }
    if (!armCommandedToTarget) {
      if ((armTarget >= ArmConstants.telescopeExtendablePosition) || 
          (telescope.getPosition() <= Constants.Telescope.safePosition)) {
            arm.rotateToPosition(armTarget);
            armCommandedToTarget = true;
      } else if (init) {
        arm.rotateToPosition(ArmConstants.telescopeExtendablePosition);
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
    if (presetTargets) {
      // complete instant command to set target positions
      return true;
    }
    if (autonomous) {
      return armCommandedToTarget && telescopeCommandedToTarget && arm.isAtTarget() && telescope.isAtTarget();
    }
    // continue holding position until cancelled in teleop
    return false;
  }
}
