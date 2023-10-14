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
  private final Arm arm = Arm.getInstance();
  private final Telescope telescope = Telescope.getInstance();
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
        // Check if robot is centered and not moving
        if (Math.abs(horizontalDegToTarget) <= LimelightConstants.substationTargetToleranceDeg && !drive.isRobotMoving()) { 
          // Too far away from substation to intake
          if (targetArea < LimelightConstants.singleSubstationIntakeTolerance) {
            if (Robot.getAllianceColor() == Alliance.Blue) {
              driveY = AutoAlignSubstationConstants.driveYSingleSubstationPower;
            } else {
              driveY = -AutoAlignSubstationConstants.driveYSingleSubstationPower;
            }
            drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
            armExtend.schedule();       
          } else {
            // Close enough to the single substation to intake
            armExtend.schedule();
            drive.stop();
          }
          if (claw.isIntakeStalling()) {
            clawStalledTimer.start();
            if ((clawStalledTimer.hasElapsed(ClawConstants.coneStalledDelay) && led.getGamePiece() == GamePiece.cone) ||
                (clawStalledTimer.hasElapsed(ClawConstants.cubeStalledDelay) && led.getGamePiece() == GamePiece.cube)) {
                  clawStalledTimer.stop();
                  clawStalledTimer.reset();
                  armRetract.schedule(); // clearance to drive away from substation
                  done = true;
            }
          }
        } else {
          // Robot close to substation, but not centered
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
        }
      } else {
        // Robot is too far away from the substation
        drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
      }
    } else {
      // Continue driving until see target again
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