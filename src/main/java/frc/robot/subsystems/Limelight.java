package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.utility.OrangeMath;

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
  boolean isTestSubsystem;
  double limeHeight;
  double limeAngle;
  boolean enabled;
  int currentPipeline = -1;
  LimelightHelpers.LimelightResults llresults;

  // the distance from where you want to calculate from
  // should always be calculated with WPI coordinates (front is positive X)
  Translation2d offset;

  private static Limelight gridLimelight;
  private static Limelight substationLimelight;

  public static Limelight getSubstationInstance() {
    if (substationLimelight == null) {
      // Measuring from front of bumpers
      // Limelight name must match limelight tool
      substationLimelight = new Limelight("limelight-load", 
      Constants.LimelightConstants.substationLimelightHeight,
      0, OrangeMath.inchesToMeters(-29.75), 
      OrangeMath.inchesToMeters(-3 - 1/4 - 3.875/2), 
      false, false, Constants.substationLimeLightEnabled);
      substationLimelight.refreshOdometry();  // prime JsonFactory to avoid long initial delay
    }
    return substationLimelight;
  }

  public static Limelight getGridInstance() {
    if (gridLimelight == null) {
      // Measuring from back of bumpers
      // Limelight name must match limelight tool
      gridLimelight = new Limelight("limelight-grid",  
      Constants.LimelightConstants.gridLimelightHeight,
      0, OrangeMath.inchesToMeters(-5.25), 
      0, true, false, Constants.gridLimeLightEnabled);
    }
    return gridLimelight;
  }

  private Limelight(String limelightName, double limelightHeightMeters, double limelightAngleDegrees,
      double xOffsetMeters, double yOffsetMeters, boolean facingBackward, boolean isTestSubsystem, boolean enabled) {
    name = limelightName;
    limeHeight = limelightHeightMeters;
    limeAngle = limelightAngleDegrees;
    offset = new Translation2d(xOffsetMeters, yOffsetMeters);
    backward = facingBackward;
    this.isTestSubsystem = isTestSubsystem;
    this.enabled = enabled;

    if (enabled) {
      table = NetworkTableInstance.getDefault().getTable(name);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      ledMode = table.getEntry("ledMode");
      camMode = table.getEntry("camMode");
      pipeline = table.getEntry("pipeline");
      activateCustomAprilTag();

      if (Constants.debug) {
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
    if (enabled) {
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

  public void refreshOdometry() {
    // parse limelight json output, takes 2.5ms
    llresults = LimelightHelpers.getLatestResults(name);
  }

  public LimelightHelpers.LimelightTarget_Fiducial getTag(double fID) {
    for (int i = 0; i < llresults.targetingResults.targets_Fiducials.length; i++) {
      if (llresults.targetingResults.targets_Fiducials[i].fiducialID == fID) {
        return llresults.targetingResults.targets_Fiducials[i];
      }
    }
    return null;  // tag ID not found
  }

  public double getHorizontalDegToTarget() {
    if (enabled) {
      return tx.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getVerticalDegToTarget() {
    if (enabled) {
      return ty.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getTargetArea() {
    if (enabled) {
      return ta.getDouble(0);
    } else {
      return 0;
    }
  }

  public boolean getTargetVisible() {
    if (enabled) {
      return tv.getDouble(0.0) == 1.0;
    } else {
      return false;
    }
  }

  public void setLed(LedMode mode) {
    if (enabled) {
      ledMode.setNumber(mode.value);
    }
  }

  public void setCamMode(CamMode mode) {
    if (enabled) {
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
    double distanceY = distanceX * Math.tan(Math.toRadians(xDeg));
    Translation2d toReturn = new Translation2d(distanceX, distanceY).plus(offset);
    if (toReturn.getX() < 0) {
      //DataLogManager.log(name + ": Tried to calculate target position, but got a negative X distance");
      toReturn = toReturn.plus(new Translation2d(-toReturn.getX(), 0));
    }
    if (backward) {
      toReturn = toReturn.rotateBy(Rotation2d.fromDegrees(180));
    }
    return toReturn;
  }

  public Translation2d getTargetPosRobotRelative() {
    if (enabled) {
      if (getTargetVisible()) {
        double yDeg = getVerticalDegToTarget();
        double xDeg = getHorizontalDegToTarget();
        int pipelineIdx = (int) pipeline.getInteger(0);
        double targetHeight = getTargetHeight(pipelineIdx, yDeg);
        if (targetHeight != -1) {
          return calcTargetPos(targetHeight, yDeg, xDeg);
        }
        //DataLogManager.log(name + ": Tried to get target pos, but pipline was invalid");
        return new Translation2d(0, 0);
      }
      //DataLogManager.log(name + ": Tried to get target pos, but no target found");
      return new Translation2d(0, 0);
    }
    //DataLogManager.log(name + ": Tried to get target pos, but limelight is disabled");
    return new Translation2d(0, 0);
  }

  public double getHorizontalDistToTarget() {
    double distanceX = (getTargetHeight(currentPipeline, getHorizontalDegToTarget()) - limeHeight) / 
                        Math.tan(Math.toRadians(getHorizontalDegToTarget() + limeAngle));
    double distanceY = distanceX * Math.tan(Math.toRadians(getVerticalDegToTarget()));

    return distanceY;
  }

  public void enableLed() {
    if (enabled) {
      setLed(LedMode.On);
    }
  }

  public void disableLed() {
    if (enabled) {
      setLed(LedMode.Off);
    }
  }

  public void activateRetroReflective() {
    switchPipeline(0);
  }

  public void activateAprilTag() {
    switchPipeline(1);
  }

  public void activateCustomAprilTag() {
    switchPipeline(2);
  }

  private void switchPipeline(int pipelineIdx) {
    if (enabled && (currentPipeline != pipelineIdx)) {
      pipeline.setNumber(pipelineIdx);
      currentPipeline = pipelineIdx;
    }
  }
}
