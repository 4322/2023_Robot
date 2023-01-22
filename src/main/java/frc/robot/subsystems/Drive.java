package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.SwerveModule;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModule = new SwerveModule[4];

  private Timer runTime = new Timer();

  private double latestVelocity;
  private double latestAcceleration;

  private final Translation2d frontLeftLocation = new Translation2d(
      Constants.DriveConstants.distWheelMetersX, Constants.DriveConstants.distWheelMetersY);
  private final Translation2d frontRightLocation = new Translation2d(
      Constants.DriveConstants.distWheelMetersX, -Constants.DriveConstants.distWheelMetersY);
  private final Translation2d backLeftLocation = new Translation2d(
      -Constants.DriveConstants.distWheelMetersX, Constants.DriveConstants.distWheelMetersY);
  private final Translation2d backRightLocation = new Translation2d(
      -Constants.DriveConstants.distWheelMetersX, -Constants.DriveConstants.distWheelMetersY);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontRightLocation,
      frontLeftLocation, backLeftLocation, backRightLocation);

  private ShuffleboardTab tab;
  private NetworkTableEntry botVelocityMag;
  private NetworkTableEntry botAccelerationMag;
  private NetworkTableEntry botVelocityAngle;
  private NetworkTableEntry botAccelerationAngle;
  private NetworkTableEntry driveXTab;
  private NetworkTableEntry driveYTab;
  private NetworkTableEntry rotateTab;

  public Drive() {

  }

  public void init() {

  }

  public enum DriveMode {
    fieldCentric(0), robotCentric(1);

    private int value;

    DriveMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private static DriveMode driveMode = DriveMode.fieldCentric;

  public static DriveMode getDrivemode() {
    return driveMode;
  }

  public void setDriveMode() {

  }

  public double getAngle() {
    return 0;
  }

  @Override
  public void periodic() {

  }

  public void resetDisplacement() {

  }

  public boolean isRobotMoving() {
    return false;
  }

  public void resetFieldCentric() {

  }

  public void drive(double driveX, double driveY, double rotate) {
    if (Constants.driveEnabled) {
      double clock = runTime.get(); // cache value to reduce CPU usage
      double[] currentAngle = new double[4];
      for (int i = 0; i < swerveModule.length; i++) {
        currentAngle[i] = swerveModule[i].getInternalRotationDegrees();
      }

      Translation2d velocityXY = new Translation2d();
      Translation2d accelerationXY = new Translation2d();
      Translation2d driveXY = new Translation2d(driveX, driveY);

      // sum wheel velocity and acceleration vectors
      for (int i = 0; i < swerveModule.length; i++) {
        double wheelAngleDegrees = currentAngle[i];
        velocityXY.plus(new Translation2d(swerveModule[i].getVelocity(),
            Rotation2d.fromDegrees(wheelAngleDegrees)));
        accelerationXY.plus(new Translation2d(swerveModule[i].getAcceleration(),
            Rotation2d.fromDegrees(wheelAngleDegrees)));
      }
      latestVelocity = velocityXY.getNorm() / 4;
      latestAcceleration = accelerationXY.getNorm() / 4;
      velocityHistory
          .removeIf(n -> (n.getTime() < clock - DriveConstants.Tip.velocityHistorySeconds));
      velocityHistory.add(new SnapshotVectorXY(velocityXY, clock));

      if (Constants.debug) {
        botVelocityMag.setDouble(latestVelocity);
        botAccelerationMag.setDouble(latestAcceleration);
        botVelocityAngle.setDouble(velocityXY.degrees());
        botAccelerationAngle.setDouble(accelerationXY.degrees());
        driveXTab.setDouble(driveX);
        driveYTab.setDouble(driveY);
        rotateTab.setDouble(rotate);
      }
    }
  }

  public boolean isAtTarget() {
    return false;
  }

  public void driveAutoRotate() {

  }

  public void resetRotatePID() {

  }

  public void updateOdometry() {

  }

  public void resetodometry() {

  }

  public void setToFieldCentric() {

  }

  public void setToBotCentric() {

  }

  public void setCoastMode() {

  }

  public void setBrakeMode() {

  }

  public void stop() {

  }

  public void setModuleStates() {

  }

  public static double boundDegrees(double angleDegrees) {
    return 0;
  }
}


// change vector --> translation2D since vector2d isn't library anymore
public class VectorXY extends Vector2d {

  public VectorXY() {
    super();
  }

  public VectorXY(double x, double y) {
    super(x, y);
  }

  public void add(Vector2d vec) {
    x += vec.x;
    y += vec.y;
  }

  public double degrees() {
    return Math.toDegrees(Math.atan2(y, x));
  }
}


public class VectorPolarDegrees extends VectorXY {

  public VectorPolarDegrees(double r, double theta) {
    x = r * Math.cos(Math.toRadians(theta));
    y = r * Math.sin(Math.toRadians(theta));
  }
}


private class SnapshotVectorXY {
  private VectorXY vectorXY;
  private double time;

  public SnapshotVectorXY(VectorXY vectorXY, double time) {
    this.vectorXY = vectorXY;
    this.time = time;
  }

  public VectorXY getVectorXY() {
    return vectorXY;
  }

  public double getTime() {
    return time;
  }

}

  // convert angle to range of +/- 180 degrees
  public static double boundDegrees(double angleDegrees) {
    double x = ((angleDegrees + 180) % 360) - 180;
    if (x < -180) {
      x += 360;
    }
    return x;
  }
}
