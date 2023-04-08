package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class AlignAssistSubstation extends CommandBase {
  private final Limelight limelight;
  private final LED led;

  public AlignAssistSubstation() {
    led = LED.getInstance();
    limelight = Limelight.getSubstationInstance();

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
      double targetArea = limelight.getTargetArea();
      double horizontalDegToTarget = limelight.getHorizontalDegToTarget();
      if (targetArea >= LimelightConstants.minLargeTargetArea) {
        if (horizontalDegToTarget <= LimelightConstants.substationTargetToleranceDeg) {
          led.setGridState(LED.GridState.aligned);
        } else if (horizontalDegToTarget > 0) {
          led.setGridState(LED.GridState.moveLeftShort);
        } else {
          led.setGridState(LED.GridState.moveRightShort);
        }
      } else {
        if (horizontalDegToTarget > 0) {
          led.setGridState(LED.GridState.moveLeft);
        } else {
          led.setGridState(LED.GridState.moveRight);
        }
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