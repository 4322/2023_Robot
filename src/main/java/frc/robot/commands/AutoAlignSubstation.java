package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.LED.GamePiece;

// TODO: Copied from AlignAssistSubstation.java as a starting point. Needs lots of work!

public class AutoAlignSubstation extends CommandBase {
  private final Limelight limelight;
  private final LED led;
  private final Drive drive;
  private final Claw claw;
  private final Arm arm = new Arm();
  private final Telescope telescope = new Telescope();

  private Timer clawStalledTimer;
  private final ClawIntake clawIntake = new ClawIntake(Claw.getInstance());

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
      led.setSubstationState(LED.SubstationState.adjusting);
      double targetArea = limelight.getTargetArea();
      double horizontalDegToTarget = limelight.getHorizontalDegToTarget()
        + LimelightConstants.substationOffsetDeg;
      
      if (targetArea >= LimelightConstants.substationMinLargeTargetArea) { // check if at correct april tag
        if (Math.abs(horizontalDegToTarget) <= LimelightConstants.substationTargetToleranceDeg) { 
          if (limelight.getTargetArea() < LimelightConstants.singleSubstationIntakeTolerance) {
            drive.drive(0, DriveConstants.driveYSingleSubstationPower, 0);
          }
          // Close enough to the single substation to intake
          if (limelight.getTargetArea() >= LimelightConstants.singleSubstationIntakeTolerance) {
            new ArmMove(arm, telescope, ArmMove.Position.loadSingleExtend, false);
            clawIntake.schedule();
          }
          if (claw.isIntakeStalling()) {
            clawStalledTimer.start();
            if ((clawStalledTimer.hasElapsed(ClawConstants.coneStalledDelay) && led.getGamePiece() == GamePiece.cone) ||
                (clawStalledTimer.hasElapsed(ClawConstants.cubeStalledDelay) && led.getGamePiece() == GamePiece.cube)) {
                  clawStalledTimer.stop();
                  clawStalledTimer.reset();
                  new ArmMove(arm, telescope, ArmMove.Position.inBot, false);
                  return;
            }
          }
        } else if (horizontalDegToTarget > 0) {
          drive.drive(-DriveConstants.driveXSingleSubstationPower, 0, 0);
        } else {
          drive.drive(DriveConstants.driveXSingleSubstationPower, 0, 0);
        }
      } else if (horizontalDegToTarget > 0) {
        drive.drive(-DriveConstants.driveXSingleSubstationPower, 0, 0);
      } else {
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