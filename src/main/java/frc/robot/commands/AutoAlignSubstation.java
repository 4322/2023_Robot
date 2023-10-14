package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
    double offCenterMeters;
    Translation2d targetDistance;  // target position relative to front center of bot

    if (limelight.getTargetVisible()) {
      limelight.refreshOdometry();
      // Use raw tag data instead of limelight calculated pose due to possible ambiguity when
      // we only see one tag because the limelight doesn't know that we are rotated to the substation.
      LimelightHelpers.LimelightTarget_Fiducial nine = limelight.getTag(9);
      LimelightHelpers.LimelightTarget_Fiducial eight = limelight.getTag(8);
      LimelightHelpers.LimelightTarget_Fiducial seven = limelight.getTag(7);

      if (eight != null) {
        targetDistance = limelight.calcTargetPos(Constants.LimelightConstants.singleSubstationAprilTagHeight,
          eight.ty, eight.tx);
        offCenterMeters = 0;
      } else if (nine != null) {
        targetDistance = limelight.calcTargetPos(Constants.LimelightConstants.singleSubstationAprilTagHeight,
          nine.ty, nine.tx);
        offCenterMeters = Constants.LimelightConstants.tagSeparationMeters;
      } else if (seven != null) {
        targetDistance = limelight.calcTargetPos(Constants.LimelightConstants.singleSubstationAprilTagHeight,
          seven.ty, seven.tx);
        offCenterMeters = Constants.LimelightConstants.tagSeparationMeters;
      } else {
        // Continue driving until see a tag again
        drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
        return;
      }
      offCenterMeters += targetDistance.getY();

      driveX = autoAlignPID.calculate(offCenterMeters, 0);
      // Check if robot is centered and not moving
      if (eight != null && Math.abs(offCenterMeters) <= LimelightConstants.substationLateralToleranceMeters && !drive.isRobotMoving()) { 
        // Too far away from substation to intake
        if (targetDistance.getX() > LimelightConstants.substationFrontToleranceMeters) {
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
        // Robot not centered
        drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
      }
    } else {
      // Continue driving until see a tag again
      drive.driveAutoRotate(driveX, driveY, targetHeadingDeg, Auto.rotateToleranceDegrees);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public boolean runsWhenDisabled() {
    return Constants.debug;
  }
}