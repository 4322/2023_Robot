package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive.DriveMode;
import frc.robot.subsystems.Drive;

public class SetDriveMode extends InstantCommand{
  private Drive drive;
  private DriveMode newDriveMode;

  public SetDriveMode(Drive drive, DriveMode newDriveMode) {
    this.drive = drive;
    this.newDriveMode = newDriveMode;
    }
    @Override
    public void initialize() {
      drive.setDriveMode(newDriveMode);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
}
