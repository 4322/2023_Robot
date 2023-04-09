package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class AlignAssistGrid extends CommandBase {
  private final Limelight limelight;
  private final LED led;

  public AlignAssistGrid() {
    led = LED.getInstance();
    limelight = Limelight.getGridInstance();

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
      
      if ((Math.abs(horizontalDegToTarget) <= LimelightConstants.gridMidTargetToleranceDeg)
          || ((Math.abs(horizontalDegToTarget) <= LimelightConstants.gridHighTargetToleranceDeg)
              && (targetArea < LimelightConstants.gridMaxHighTargetArea))) {
        led.setGridState(LED.GridState.aligned);
      } else if (horizontalDegToTarget > 0) {
        led.setGridState(LED.GridState.moveLeft);
      } else {
        led.setGridState(LED.GridState.moveRight);
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
