package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class DriveBreakout extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new DriveBreakout.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;

  public DriveBreakout(Drive drivesubsystem) {
    drive = drivesubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Command ternimates instantly, in order to break the drive out of rotation mode
  // by making another command require it

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("Tried to break out of directional rotation");
  }

}
