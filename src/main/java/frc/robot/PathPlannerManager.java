package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

class PathPlannerManager {

  Drive drive;

  SwerveAutoBuilder builder;
  HashMap<String, Command> eventMap;

  PathPlannerManager(Drive driveSubsystem) {

    drive = driveSubsystem;

    builder = new SwerveAutoBuilder(
      drive::getPose2d, 
      drive::resetOdometry, 
      drive.getKinematics(),
      new PIDConstants(
        DriveConstants.Trajectory.PIDXY.kP, 
        DriveConstants.Trajectory.PIDXY.kI,
        DriveConstants.Trajectory.PIDXY.kD
      ),
      new PIDConstants(
        DriveConstants.Trajectory.PIDR.kP, 
        DriveConstants.Trajectory.PIDR.kI,
        DriveConstants.Trajectory.PIDR.kD
      ),
      drive::setModuleStates, 
      eventMap, 
      drive
    );

  }

  PPSwerveControllerCommand generateSingleCommand(String pathName, boolean reversed) {

    PathConstraints constraints = new PathConstraints(DriveConstants.autoMaxSpeedMetersPerSecond,
        DriveConstants.autoMaxAccelerationMetersPerSec2);
        
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, constraints);

    PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
        trajectory, 
        drive::getPose2d, 
        drive.getKinematics(), 
        new PIDController(
          DriveConstants.Trajectory.PIDXY.kP, 
          DriveConstants.Trajectory.PIDXY.kI, 
          DriveConstants.Trajectory.PIDXY.kD
        ),
        new PIDController(
          DriveConstants.Trajectory.PIDXY.kP, 
          DriveConstants.Trajectory.PIDXY.kI, 
          DriveConstants.Trajectory.PIDXY.kD
        ),
        new PIDController(
          DriveConstants.Trajectory.PIDR.kP, 
          DriveConstants.Trajectory.PIDR.kI, 
          DriveConstants.Trajectory.PIDR.kD
        ),
        drive::setModuleStates,
        drive
    );

    return swerveCommand;

  }

}
