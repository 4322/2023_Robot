package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveRotateWheels extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive drive;
  private double abortSec;
  private Timer timer = new Timer();

  public AutoDriveRotateWheels(Drive drivesubsystem, double abortSeconds) {
    drive = drivesubsystem;
    abortSec = abortSeconds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(0, 0, 0.001);
    timer.reset();
    timer.start();
  }

  // No execute because nothing to do

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(abortSec);
  }
}
