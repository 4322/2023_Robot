package frc.robot.commands;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveAlignTag extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive drive;
  private final Limelight limelight;
  private Translation2d targetPosRobotRelative;
  private double yTolerance;
  private double abortSec;
  private Timer timer = new Timer();

  public DriveAlignTag(Drive drivesubsystem, Limelight limelightsubsystem, double alignTolerance, double abortSeconds) {
    drive = drivesubsystem;
    limelight = limelightsubsystem;
    abortSec = abortSeconds;
    yTolerance = alignTolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (limelight.getTargetVisible()) {
      if (limelight.getHorizontalDegToTarget() > 0 && 
      limelight.getHorizontalDegToTarget() > LimelightConstants.aprilTagHorizontalTolerance) {

      } else if (limelight.getHorizontalDegToTarget() < 0 &&
      Math.abs(limelight.getHorizontalDegToTarget()) > LimelightConstants.aprilTagHorizontalTolerance) {
        
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(abortSec) || (Math.abs(targetPosRobotRelative.getY()) < yTolerance);
  }
}
