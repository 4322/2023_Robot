package frc.robot.commands;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.DriveConstants.Auto;
import frc.robot.Constants.DriveConstants.AutoAlignSubstationConstants;
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
  private PIDController autoAlignPID;

  private Timer clawStalledTimer;
  private final ArmMove armExtend = new ArmMove(arm, telescope, ArmMove.Position.loadSingleExtend, false);
  private final ArmMove armRetract = new ArmMove(arm, telescope, ArmMove.Position.loadSingleRetract, false);

  private double driveX = AutoAlignSubstationConstants.driveXMax;
  private double driveY = 0.0;

  private Double targetHeadingDeg = null;

  private boolean done;

  public AutoAlignSubstation(Drive driveSubsystem) {
    led = LED.getInstance();
    limelight = Limelight.getSubstationInstance();
    drive = driveSubsystem;
    claw = Claw.getInstance();

    autoAlignPID.setP(AutoAlignSubstationConstants.kP);
    autoAlignPID.setD(AutoAlignSubstationConstants.kD);

    addRequirements(limelight, drive);
  }

  @Override
  public void initialize() {
    clawStalledTimer.stop();
    clawStalledTimer.reset();
    done = false;

    switch (Robot.getAllianceColor()) {
      case Blue:
        targetHeadingDeg = 90.0;
        break;
      case Red:
        targetHeadingDeg = -90.0;
        break;
      default:
        // unknown direction to single substation
        targetHeadingDeg = null;
        done = true;
        break;
    }
  }

  @Override
  public void execute() {
    led.setSubstationState(LED.SubstationState.adjusting);
      double targetArea = limelight.getTargetArea();
      double horizontalDegToTarget = limelight.getHorizontalDegToTarget()
        + LimelightConstants.substationOffsetDeg;

    if (limelight.getTargetVisible()) {
      driveX = autoAlignPID.calculate(limelight.getHorizontalDistToTarget(), 0);
      if (targetArea >= LimelightConstants.substationMinLargeTargetArea) { // check if at correct april tag
        if (Math.abs(horizontalDegToTarget) <= LimelightConstants.substationTargetToleranceDeg) { 
          if (limelight.getTargetArea() < LimelightConstants.singleSubstationIntakeTolerance) {
            // Check if robot is moving to make sure robot isn't overshooting
            if (drive.isRobotMoving()) {
              if (Robot.getAllianceColor() == Alliance.Blue) {
                driveY = -AutoAlignSubstationConstants.driveYSingleSubstationPower;
              } else {
                driveY = AutoAlignSubstationConstants.driveYSingleSubstationPower;
              }
              drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
              armExtend.schedule();       
            }
          }
          // Close enough to the single substation to intake
          if (limelight.getTargetArea() >= LimelightConstants.singleSubstationIntakeTolerance) {
            drive.stop();
          }
          if (claw.isIntakeStalling()) {
            clawStalledTimer.start();
            if ((clawStalledTimer.hasElapsed(ClawConstants.coneStalledDelay) && led.getGamePiece() == GamePiece.cone) ||
                (clawStalledTimer.hasElapsed(ClawConstants.cubeStalledDelay) && led.getGamePiece() == GamePiece.cube)) {
                  clawStalledTimer.stop();
                  clawStalledTimer.reset();
                  armRetract.schedule();
                  done = true;
            }
          }
        } else if (horizontalDegToTarget > 0) {
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
        } else {
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
        }
      } else if (horizontalDegToTarget > 0) {
        drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
      } else {
        drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
      }
    } else {
      drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (done) {
      return true;
    }
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