package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LED.GamePiece;

// TODO: Copied from AlignAssistSubstation.java as a starting point. Needs lots of work!

public class AutoAlignSubstation extends CommandBase {
  private final Limelight limelight;
  private final LED led;
  private final Drive drive;
  private Claw claw;
  private Timer clawStalledTimer;

  public AutoAlignSubstation(Drive driveSubsystem) {
    led = LED.getInstance();
    limelight = Limelight.getSubstationInstance();
    drive = driveSubsystem;
    claw = Claw.getInstance();

    addRequirements(limelight, drive);
  }

  @Override
  public void initialize() {
    clawStalledTimer.stop();
    clawStalledTimer.reset();
  }

  @Override
  public void execute() {
    if (limelight.getTargetVisible()) {
      double targetArea = limelight.getTargetArea();
      double horizontalDegToTarget = limelight.getHorizontalDegToTarget()
        + LimelightConstants.substationOffsetDeg;
      
      if (targetArea >= LimelightConstants.substationMinLargeTargetArea) { // check if at correct april tag
        if (Math.abs(horizontalDegToTarget) <= LimelightConstants.substationTargetToleranceDeg) {
          led.setSubstationState(LED.SubstationState.adjusting); 
          if (limelight.getTargetArea() < LimelightConstants.singleSubstationIntakeTolerance) {
            drive.drive(0, DriveConstants.driveYSingleSubstationPower, 0);
          }
          if (limelight.getTargetArea() >= LimelightConstants.singleSubstationIntakeTolerance) {
            new ClawIntake(claw);
          }
          if (claw.isIntakeStalling()) {
            clawStalledTimer.start();
            if ((clawStalledTimer.hasElapsed(ClawConstants.coneStalledDelay) && led.getGamePiece() == GamePiece.cone) ||
                (clawStalledTimer.hasElapsed(ClawConstants.cubeStalledDelay) && led.getGamePiece() == GamePiece.cube)) {
                  clawStalledTimer.stop();
                  clawStalledTimer.reset();
                  new DriveManual(drive, null);
            }
          }
        } else if (horizontalDegToTarget > 0) {
          led.setSubstationState(LED.SubstationState.adjusting);
          drive.drive(-DriveConstants.driveXSingleSubstationPower, 0, 0);
        } else {
          led.setSubstationState(LED.SubstationState.adjusting);
          drive.drive(DriveConstants.driveXSingleSubstationPower, 0, 0);
        }
      } else if (horizontalDegToTarget > 0) {
        led.setSubstationState(LED.SubstationState.adjusting);
        drive.drive(-DriveConstants.driveXSingleSubstationPower, 0, 0);
      } else {
        led.setSubstationState(LED.SubstationState.adjusting);
        drive.drive(DriveConstants.driveXSingleSubstationPower, 0, 0);
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