package frc.utility;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;

class PathPlannerUtil {

  PPSwerveControllerCommand generateCommand(Drive drive, String pathName, boolean reversed) {

    PathConstraints constraints = new PathConstraints(DriveConstants.autoMaxSpeedMetersPerSecond,
        DriveConstants.autoMaxAccelerationMetersPerSec2);
        
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, constraints);

    PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
        trajectory, 
        drive::getPose2d, 
        drive.getKinematics(), 
        new PIDController(
          DriveConstants.Trajectory.PIDX.kP, 
          DriveConstants.Trajectory.PIDX.kI, 
          DriveConstants.Trajectory.PIDX.kD
        ),
        new PIDController(
          DriveConstants.Trajectory.PIDY.kP, 
          DriveConstants.Trajectory.PIDY.kI, 
          DriveConstants.Trajectory.PIDY.kD
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
