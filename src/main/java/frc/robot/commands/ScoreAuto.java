package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class ScoreAuto extends CommandBase {
  private scoringStates currentMode;
  private Drive drive;
  private Arm arm;
  private Claw claw;
  private Limelight limelight;

  private Timer timer = new Timer();

  public enum scoringStates {
    firstStop, startPath, drivingPath, score, done, abort;
  }

  public ScoreAuto(Drive drive, Arm armSubsystem, Claw clawSubsystem, Limelight limelight) {
    this.drive = drive;
    arm = armSubsystem;
    claw = clawSubsystem;
    this.limelight = limelight;

    addRequirements(arm, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentMode = scoringStates.firstStop;
    timer.reset();
    timer.start();
    drive.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentMode) {
      case firstStop:
        if (timer.hasElapsed(1.0)) {
          currentMode = scoringStates.startPath;
        }
      case startPath:
        // add path planner calculations
        // need to change drivex, drivey, and rotate variables
        drive.drive(Constants.DriveConstants.driveX, Constants.DriveConstants.driveY,
            Constants.DriveConstants.rotate);
        PathPlannerTrajectory path = PathPlanner.generatePath(
            new PathConstraints(Constants.AutoScore.maxVelocity,
                Constants.AutoScore.maxAcceleration),
            new PathPoint(
                new Translation2d(
                    limelight.getTargetPosRobotRelative().getX() * Math.cos(drive.getAngle()),  // X start position
                    limelight.getTargetPosRobotRelative().getY() * Math.sin(drive.getAngle())), // Y start position
                Rotation2d.fromDegrees(0), // start heading (direction of travel)
                Rotation2d.fromDegrees(drive.getAngle())), // start holonomic rotation
            new PathPoint(
                new Translation2d(Constants.AutoScore.endPosX, Constants.AutoScore.endPosY), // end position
                Rotation2d.fromDegrees(Constants.AutoScore.endDirection), // end heading(direction of travel)
                Rotation2d.fromDegrees(Constants.AutoScore.scoreAngle)) // end holonomic rotation
        );
        currentMode = scoringStates.drivingPath;
      case drivingPath:
        if (limelight.getTargetPosRobotRelative()
            .getY() == Constants.LimelightConstants.distanceToTargetInches) {
          currentMode = scoringStates.score;
        }
      case score:
        // run claw outtake
        arm.rotateToPosition(Constants.ArmConstants.MidScoringPosition);
        claw.outtake();
        // give time for cone or cube to exit grabber
        if (timer.hasElapsed(0.5)) {
          currentMode = scoringStates.done;
        }
      case abort:
        drive.stop();
        arm.stop();
        claw.stop();
        currentMode = scoringStates.done;
      case done:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    arm.stop();
    claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
