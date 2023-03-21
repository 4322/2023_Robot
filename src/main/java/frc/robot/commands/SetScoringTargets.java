package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class SetScoringTargets extends InstantCommand {
  private Arm arm;
  private Telescope telescope;
  private double armTarget;
  private double telescopeTarget;

  // all parameters
  public SetScoringTargets(Arm arm, Telescope telescope, double armTarget, double telescopeTarget) {
    this.arm = arm;
    this.telescope = telescope;
    this.armTarget = armTarget;
    this.telescopeTarget = telescopeTarget;

    // don't interrupt existing command because the trigger command won't restart
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setScoringTarget(armTarget);
    telescope.setScoringTarget(telescopeTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
