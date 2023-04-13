package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetScoringPosition extends InstantCommand {
  private ArmMove.Position pos;

  // all parameters
  public SetScoringPosition(ArmMove.Position pos) {
    this.pos = pos;

    // don't interrupt existing command because the trigger command won't restart
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmMove.setScorePreset(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
