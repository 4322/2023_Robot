package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
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
  NetworkTableEntry distanceToTarget;
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
        distanceToTarget.setDouble(getDistance());
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
    Off(1),
    Blink(2),
    On(3);

    private int value;

    LedMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  public enum CamMode {
    VisionProcessor,
    DriverCamera;
  }

  //Formula Referenced From: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public double getDistance() {
    double distance = 0;
    
    if (getTargetVisible()) {
    double angleToTarget = LimelightConstants.limelightAngle + getVerticalDegToTarget();
    distance = (LimelightConstants.targetHeight - LimelightConstants.limelightHeight) /
    Math.tan(Math.toRadians(angleToTarget));
    }

    return distance;
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
}