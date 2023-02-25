package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase{
  private Drive drive;
  String direction;

  public AutoBalance(Drive driveSubsystem, String direction) {
    drive = driveSubsystem;
    this.direction = direction;

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (direction.equals("Forward")) { // Coming from direction of drive stations
      drive.drive(Constants.DriveConstants.autoBalanceVelocity, 0, 0); // not rotating or moving side to side
      if (drive.getAngle() >= Constants.DriveConstants.chargeStationOffAngle) {
        if (drive.getAngle() <= Constants.DriveConstants.chargeStationOffAngle) {
          drive.stop();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
