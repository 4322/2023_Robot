package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class AlignAssistGrid extends CommandBase {
  private final Limelight limelight;
  private final LED led;
  private Translation2d targetPos;
  private double horizontalOffset;

  public AlignAssistGrid() {
    led = LED.getInstance();
    limelight = Limelight.getGridInstance();

    addRequirements(limelight);
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
      horizontalOffset = targetPos.getY();
      if (Math.abs(horizontalOffset) <= LimelightConstants.gridTargetToleranceMeters) {
        led.setGridState(LED.GridState.aligned);
      } else if (Math.abs(targetPos.getX()) <= LimelightConstants.gridTargetCloseMeters) {
        if (horizontalOffset > 0) {
          led.setGridState(LED.GridState.moveLeft);
        } else {
          led.setGridState(LED.GridState.moveRight);
        }
      } else {
        led.setGridState(LED.GridState.off);
      }
    } else {
      led.setGridState(LED.GridState.off);
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
