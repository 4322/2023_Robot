package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightManager;
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

  // All distances use WPILib coordinates (where x is perpendicular to the target and y
  // is parallel to the target)
  public Translation2d getTargetPosRobotRelative() {
    if (Constants.limeLightEnabled) {
      if (getTargetVisible()) {
        double yDeg = getVerticalDegToTarget();
        double xDeg = getHorizontalDegToTarget();
        int pipelineIdx = (int) pipeline.getInteger(0);

        double targetHeight = LimelightManager.getTargetHeight(pipelineIdx, yDeg);
        double angleToTarget = LimelightConstants.limelightAngle + yDeg;

        if (targetHeight != -1) {
          return LimelightManager.calcTargetPos(targetHeight, angleToTarget, xDeg);
        } else {
          DataLogManager.log("Tried to get target pos, but pipline was invalid");
          return new Translation2d(0, 0);
        }
      }
      DataLogManager.log("Tried to get target pos, but no target found");
      return new Translation2d(0, 0);
    }
    DataLogManager.log("Tried to get target pos, but limelight is disabled");
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
