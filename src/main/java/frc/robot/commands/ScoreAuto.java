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
import frc.robot.commands.ClawOuttake;

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
        if (timer.hasElapsed(1.0)){
          currentMode = scoringStates.startPath;
        }
      case startPath:
        // add path planner calculations
        // need to change drivex, drivey, and rotate variables
        drive.drive(Constants.DriveConstants.driveX, Constants.DriveConstants.driveY,
        Constants.DriveConstants.rotate);
        PathPlannerTrajectory path = PathPlanner.generatePath (
          new PathConstraints(0, 0),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(drive.getAngle())), // position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(Math.cos(drive.getAngle()), Math.sin(drive.getAngle())), Rotation2d.fromDegrees(Constants.DriveConstants.AutoScore.endDirection), Rotation2d.fromDegrees(Constants.DriveConstants.AutoScore.scoreAngle))  // position, heading(direction of travel), holonomic rotation
        );
          
        
        currentMode = scoringStates.drivingPath;
      case drivingPath:
        if (limelight.getTargetPosRobotRelative().getY() == Constants.LimelightConstants.distanceToTargetInches) {
          currentMode = scoringStates.score;
        }
      case score:
        // run claw outtake
        arm.rotateToPosition(Constants.ArmConstants.MidScoringPosition);
        claw.outtake();
        currentMode = scoringStates.done;
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
