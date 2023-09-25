package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class SetArmPreset extends InstantCommand {
  private ArmMove.Position pos;

  // all parameters
  public SetArmPreset(Drive drive, ArmMove.Position pos) {
    this.pos = pos;

    // interrupt existing auto-rotate command so the trigger will work when next pressed
    addRequirements(drive);
    
    // don't interrupt existing arm commands because the trigger command won't restart
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmMove.setArmPreset(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
