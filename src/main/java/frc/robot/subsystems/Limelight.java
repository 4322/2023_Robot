package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  GenericEntry distanceToTargetX;
  GenericEntry distanceToTargetY;
  GenericEntry targetVisible;

  String name;
  boolean backward;
  boolean test;
  double limeHeight;
  double limeAngle;

  // the distance from where you want to calculate from
  // should always be calculated with WPI coordinates (front is positive X)
  Translation2d offset;

  // TODO: make this nicer with default params
  public Limelight(String limelightName, double limelightHeightMeters, double limelightAngleDegrees,
      double xOffsetMeters, double yOffsetMeters, boolean facingBackward, boolean isTestSubsystem) {
    if (Constants.limeLightEnabled) {
      name = limelightName;
      limeHeight = limelightHeightMeters;
      limeAngle = limelightAngleDegrees;
      offset = new Translation2d(xOffsetMeters, yOffsetMeters);
      backward = facingBackward;
      test = isTestSubsystem;

      table = NetworkTableInstance.getDefault().getTable(name);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      ledMode = table.getEntry("ledMode");
      camMode = table.getEntry("camMode");
      pipeline = table.getEntry("pipeline");

      if (Constants.debug && !test) {
        tab = Shuffleboard.getTab(name);
        targetVisible = tab.add("Target Visible", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0).getEntry();
        distanceToTargetX = tab.add("Target X", 0).withPosition(0, 1).getEntry();
        distanceToTargetY = tab.add("Target Y", 0).withPosition(0, 2).getEntry();
      }
    }
  }

  @Override
  public void periodic() {
    if (Constants.limeLightEnabled) {
      if (Constants.debug) {
        boolean visible = getTargetVisible();
        targetVisible.setBoolean(visible);
        if (visible) {
          Translation2d targetPos = getTargetPosRobotRelative();
          distanceToTargetX.setDouble(targetPos.getX());
          distanceToTargetY.setDouble(targetPos.getY());
        }
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

  // All distances use WPILib coordinates (where x is perpendicular to the target and y
  // is parallel to the target)
  public double getTargetHeight(int pipelineIdx, double yDeg) {
    if (LimelightConstants.tapePipelines.contains(pipelineIdx)) {
      if (yDeg > LimelightConstants.tapeTargetHeightThresholdDeg) {
        return LimelightConstants.highTapeHeight;
      } else {
        return LimelightConstants.middleTapeHeight;
      }
    } else if (LimelightConstants.tagPipelinesHeights.containsKey(pipelineIdx)) {
      return LimelightConstants.tagPipelinesHeights.get(pipelineIdx);
    }
    return -1; // invalid pipeline
  }

  // Angles in degrees
  // distance y formula referenced from:
  // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public Translation2d calcTargetPos(double targetHeight, double yDeg, double xDeg) {
    double distanceX = (targetHeight - limeHeight) / Math.tan(Math.toRadians(yDeg + limeAngle));
    double distanceY = distanceX / Math.tan(Math.toRadians(xDeg));
    Translation2d toReturn = new Translation2d(distanceX, distanceY).plus(offset);
    if (toReturn.getX() < 0) {
      DataLogManager
          .log(name + ": Tried to calculate target position, but got a negative X distance");
      toReturn = toReturn.plus(new Translation2d(-toReturn.getX(), 0));
    }
    if (backward) {
      toReturn = toReturn.rotateBy(Rotation2d.fromDegrees(180));
    }
    if (test) {
      DataLogManager
          .log("Calculated Position: x = " + toReturn.getX() + ". y = " + toReturn.getY());
    }
    return toReturn;
  }

  public Translation2d getTargetPosRobotRelative() {
    if (Constants.limeLightEnabled) {
      if (getTargetVisible()) {
        double yDeg = getVerticalDegToTarget();
        double xDeg = getHorizontalDegToTarget();
        int pipelineIdx = (int) pipeline.getInteger(0);
        double targetHeight = getTargetHeight(pipelineIdx, yDeg);
        if (targetHeight != -1) {
          return calcTargetPos(targetHeight, yDeg, xDeg);
        }
        DataLogManager.log(name + ": Tried to get target pos, but pipline was invalid");
        return new Translation2d(0, 0);
      }
      DataLogManager.log(name + ": Tried to get target pos, but no target found");
      return new Translation2d(0, 0);
    }
    DataLogManager.log(name + ": Tried to get target pos, but limelight is disabled");
    return new Translation2d(0, 0);
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
