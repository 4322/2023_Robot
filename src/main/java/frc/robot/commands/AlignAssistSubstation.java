package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class AlignAssistSubstation extends CommandBase {
  private final Limelight limelight;
  private final LED led = LED.getInstance();
  private Translation2d targetPos;
  private double horizontalOffset;

  public AlignAssistSubstation(Limelight substationLimelight) {
    limelight = substationLimelight;

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
      if (Math.abs(targetPos.getX()) < LimelightConstants.assistedAlignStartDistanceMeters) {
        if (Math.abs(horizontalOffset) < LimelightConstants.horizontalAlignToleranceMeters) {
          led.setSubstationState(LED.SubstationState.aligned);
        } else if (horizontalOffset > LimelightConstants.horizontalAlignToleranceMeters) {
          led.setSubstationState(LED.SubstationState.moveLeft);
        } else {
          led.setSubstationState(LED.SubstationState.moveRight);
        }
      } else {
        led.setSubstationState(LED.SubstationState.off);
      }
    } else {
      led.setSubstationState(LED.SubstationState.off);
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