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

  @Override
  public void execute() {
    if (limelight.getTargetVisible()) {
      double targetArea = limelight.getTargetArea();
      double horizontalDegToTarget = limelight.getHorizontalDegToTarget();
      
      if (targetArea >= LimelightConstants.substationMinLargeTargetArea) {
        if (Math.abs(horizontalDegToTarget) <= LimelightConstants.substationTargetToleranceDeg) {
          led.setSubstationState(LED.SubstationState.aligned);
        } else if (horizontalDegToTarget > 0) {
          led.setSubstationState(LED.SubstationState.moveLeftShort);
        } else {
          led.setSubstationState(LED.SubstationState.moveRightShort);
        }
      } else if (horizontalDegToTarget > 0) {
        led.setSubstationState(LED.SubstationState.moveLeft);
      } else {
        led.setSubstationState(LED.SubstationState.moveRight);
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