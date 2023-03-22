package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.LimelightConstants;

// All distances use WPILib coordinates (where x is perpendicular to the target and y
// is parallel to the target)
public class LimelightManager {

  public static double getTargetHeight(int pipelineIdx, double yDeg) {

    if (LimelightConstants.tapePipelines.contains(pipelineIdx)) {
      if (yDeg > LimelightConstants.targetHeightThresholdDeg) {
        return LimelightConstants.highTapeHeight;
      } else {
        return LimelightConstants.middleTapeHeight;
      }
    } else if (LimelightConstants.tagPipelinesHeights.containsKey(pipelineIdx)) {
      return LimelightConstants.tagPipelinesHeights.get(pipelineIdx);
    }
    return -1; // invalid pipeline, return empty translation
  }

  // Angles in degrees
  // distance y formula referenced from:
  // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public static Translation2d calcTargetPos(double targetHeight, double verticalAngleToTarget,
      double horizontalAngleToTarget) {
    double distanceX = (targetHeight - LimelightConstants.limelightHeight)
        / Math.tan(Math.toRadians(verticalAngleToTarget));
    double distanceY = distanceX / Math.tan(Math.toRadians(horizontalAngleToTarget));
    Translation2d toReturn = new Translation2d(distanceX, distanceY);
    if (toReturn.getX() < 0) {
      DataLogManager.log("Tried to calculate target position, but got a negative X distance");
      return new Translation2d();
    }
    return toReturn;
  }

};