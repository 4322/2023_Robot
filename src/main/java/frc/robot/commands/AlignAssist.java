package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class AlignAssist extends CommandBase {
  private LED LED;
  private Limelight limelight;
  private Translation2d targetPos;
  private double horizontalOffset;

  public AlignAssist(LED ledSubsystem, Limelight gridLimelight) {
    LED = ledSubsystem;
    limelight = gridLimelight;
  }

  @Override
  public void initialize() {

  }

  // Red: too far to the left (move right)
  // Blue: too far to the right (move left)

  @Override
  public void execute() {
    if (limelight.getTargetVisible()) {

      targetPos = limelight.getTargetPosRobotRelative();
      horizontalOffset = targetPos.getX();

      if (targetPos.getY() < LimelightConstants.assistedAlignStartDistanceMeters) {
        if (Math.abs(horizontalOffset) < LimelightConstants.horizontalAlignToleranceMeters) {
          LED.green();
        } else if (horizontalOffset > LimelightConstants.horizontalAlignToleranceMeters) {
          LED.red();
        } else {
          LED.blue();
        }
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
