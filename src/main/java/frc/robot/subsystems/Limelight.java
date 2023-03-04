package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry ledMode;
  NetworkTableEntry camMode;
  NetworkTableEntry pipeline;

  // SHUFFLEBOARD
  ShuffleboardTab tab;
  NetworkTableEntry distanceToTargetX;
  NetworkTableEntry distanceToTargetY;
  NetworkTableEntry targetVisible;

  public Limelight() {
    if (Constants.limeLightEnabled) {
      table = NetworkTableInstance.getDefault().getTable("limelight");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      ledMode = table.getEntry("ledMode");
      camMode = table.getEntry("camMode");
      pipeline = table.getEntry("pipeline");
    }
  }

  @Override
  public void periodic() {
    if (Constants.limeLightEnabled) {
      SmartDashboard.putBoolean("Target Visible", getTargetVisible());
      if (Constants.debug) {
        Translation2d targetPos = getTargetPosRobotRelative();
        distanceToTargetX.setDouble(targetPos.getX());
        distanceToTargetY.setDouble(targetPos.getY());
        targetVisible.setBoolean(getTargetVisible());
      }
    }
  }

  public double getHorizontalDegToTarget() {
    if (Constants.limeLightEnabled) {
      return tx.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getVerticalDegToTarget() {
    if (Constants.limeLightEnabled) {
      return ty.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getTargetArea() {
    if (Constants.limeLightEnabled) {
      return ta.getDouble(0);
    } else {
      return 0;
    }
  }

  public boolean getTargetVisible() {
    if (Constants.limeLightEnabled) {
      return tv.getDouble(0.0) == 1.0;
    } else {
      return false;
    }
  }

  public void setLed(LedMode mode) {
    if (Constants.limeLightEnabled) {
      ledMode.setNumber(mode.value);
    }
  }

  public void setCamMode(CamMode mode) {
    if (Constants.limeLightEnabled) {
      if (mode == CamMode.VisionProcessor) {
        camMode.setNumber(0);
      } else if (mode == CamMode.DriverCamera) {
        camMode.setNumber(1);
      }
    }
  }

  public enum LedMode {
    Off(1), Blink(2), On(3);

    private int value;

    LedMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  public enum CamMode {
    VisionProcessor, DriverCamera;
  }

  // distance y formula referenced from:
  // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  // All distances assume coordinate plane from top view (where y is perpendicular to the grid and x
  // is parallel to the grid)
  public Translation2d getTargetPosRobotRelative() {
    if (Constants.limeLightEnabled) {
      double distanceX = 0;
      double distanceY = 0;
      double yDeg = getVerticalDegToTarget();
      double xDeg = getHorizontalDegToTarget();
      int pipelineIdx = (int) pipeline.getInteger(0); // Note: long is a longer integer
      List<Integer> tapeList = Arrays.asList(LimelightConstants.tapePipelines);

      if (getTargetVisible()) {

        double angleToTarget = LimelightConstants.limelightAngle + yDeg;
        
        if (tapeList.contains(pipelineIdx)) {

          if (yDeg > LimelightConstants.targetHeightThresholdDeg) {
            distanceY = (LimelightConstants.highTapeHeight - LimelightConstants.limelightHeight)
              / Math.tan(Math.toRadians(angleToTarget));
          } else {
            distanceY = (LimelightConstants.middleTapeHeight - LimelightConstants.limelightHeight)
              / Math.tan(Math.toRadians(angleToTarget));
          } 

        } else {
          distanceY = (LimelightConstants.gridAprilTagHeight - LimelightConstants.limelightHeight)
            / Math.tan(Math.toRadians(angleToTarget));
        }

        distanceX = distanceY * Math.tan(Math.toRadians(xDeg));

      }

      return new Translation2d(distanceX, distanceY);
    } else {
      return null;
    }
  }

  public void enableLed() {
    if (Constants.limeLightEnabled) {
      setLed(LedMode.On);
    }
  }

  public void disableLed() {
    if (Constants.limeLightEnabled) {
      setLed(LedMode.Off);
    }
  }

  public void switchPipeline(int pipelineIdx) {
    if (Constants.limeLightEnabled) {
      pipeline.setNumber(pipelineIdx);
    }
  }
}
