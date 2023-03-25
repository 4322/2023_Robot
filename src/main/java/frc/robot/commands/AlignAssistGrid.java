package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class AlignAssistGrid extends CommandBase {
  private Limelight limelight;
  private final LED led = LED.getInstance();
  private Translation2d targetPos;
  private double horizontalOffset;

  private enum TargetState {
    searching, found
  }

  private TargetState targetStatus;

  public AlignAssistGrid(Limelight gridLimelight) {
    limelight = gridLimelight;

    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    targetStatus = TargetState.searching;
  }

  // Red: too far to the left (move right)
  // Blue: too far to the right (move left)

  @Override
  public void execute() {
    switch (targetStatus) {
      case searching:
        if (limelight.getTargetVisible()) {
          if (Math.abs(limelight.getTargetPosRobotRelative()
              .getX()) < LimelightConstants.assistedAlignStartDistanceMeters) {
            // Broken into 2 if statements so that target is not calculated without a valid target
            targetStatus = TargetState.found;
          }
        }
        break;
      case found:
        if (limelight.getTargetVisible()) {
          targetPos = limelight.getTargetPosRobotRelative();
          horizontalOffset = targetPos.getY();
          if (Math.abs(targetPos.getX()) < LimelightConstants.assistedAlignStartDistanceMeters) {
            if (Math.abs(horizontalOffset) < LimelightConstants.horizontalAlignToleranceMeters) {
              led.setGridState(LED.GridState.aligned);
            } else if (horizontalOffset > LimelightConstants.horizontalAlignToleranceMeters) {
              led.setGridState(LED.GridState.moveLeft);
            } else {
              led.setGridState(LED.GridState.moveRight);
            }
          } else {
            led.setGridState(LED.GridState.off);
            targetStatus = TargetState.searching;
          }
        } else {
          led.setGridState(LED.GridState.off);
          targetStatus = TargetState.searching;
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

  @Override
  public boolean runsWhenDisabled() {
    if (Constants.debug) {
      return true;
    } else {
      return false;
    }
  }
}
