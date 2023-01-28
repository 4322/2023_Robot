package frc.robot.subsystems;

import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.SwerveModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModule = new SwerveModule[4];

  private AHRS gyro;
  private PIDController rotPID;

  private Timer runTime = new Timer();

  private double latestVelocity;
  private double latestAcceleration;

  private ArrayList<SnapshotVectorXY> velocityHistory = new ArrayList<SnapshotVectorXY>();

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

  private SwerveDriveOdometry odometry;

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
    if (Constants.driveEnabled) {

      rotPID = new PIDController(DriveConstants.autoRotkP, 0, DriveConstants.autoRotkD);

      if (Constants.gyroEnabled) {
        gyro = new AHRS(SPI.Port.kMXP);
    

        // wait for first gyro reading to be received
        try {
          Thread.sleep(250);
        } catch (InterruptedException e) {
        }

        resetFieldCentric(0);
      } else {
        setDriveMode(DriveMode.frontCamCentric);
      }

      if (Constants.driveEnabled) {
        swerveModule[WheelPosition.FRONT_RIGHT.wheelNumber] = new SwerveModule(
            DriveConstants.frontRightRotationID, DriveConstants.frontRightDriveID,
            WheelPosition.FRONT_RIGHT, DriveConstants.frontRightEncoderID);
        swerveModule[WheelPosition.FRONT_LEFT.wheelNumber] = new SwerveModule(DriveConstants.frontLeftRotationID,
            DriveConstants.frontLeftDriveID,
            WheelPosition.FRONT_LEFT, DriveConstants.frontLeftEncoderID);
        swerveModule[WheelPosition.BACK_RIGHT.wheelNumber] = new SwerveModule(DriveConstants.rearRightRotationID,
            DriveConstants.rearRightDriveID,
            WheelPosition.BACK_RIGHT, DriveConstants.rearRightEncoderID);
        swerveModule[WheelPosition.BACK_LEFT.wheelNumber] = new SwerveModule(DriveConstants.rearLeftRotationID,
            DriveConstants.rearLeftDriveID,
            WheelPosition.BACK_LEFT, DriveConstants.rearLeftEncoderID);
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

        for (SwerveModule module : swerveModule) {
          module.init();
        }
      }

      ram.setTolerance(DriveConstants.poseError);

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Drivebase");

        rotErrorTab = tab.add("Rot Error", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

        rotSpeedTab = tab.add("Rotation Speed", 0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

        rotkP = tab.add("Rotation kP", DriveConstants.autoRotkP)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();

        rotkD = tab.add("Rotation kD", DriveConstants.autoRotkD)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();

        roll = tab.add("Roll", 0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();

        pitch = tab.add("Pitch", 0)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();

        botVelocityMag = tab.add("Bot Vel Mag", 0)
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();

        botAccelerationMag = tab.add("Bot Acc Mag", 0)
            .withPosition(3, 1)
            .withSize(1, 1)
            .getEntry();

        botVelocityAngle = tab.add("Bot Vel Angle", 0)
            .withPosition(4, 0)
            .withSize(1, 1)
            .getEntry();

        botAccelerationAngle = tab.add("Bot Acc Angle", 0)
            .withPosition(4, 1)
            .withSize(1, 1)
            .getEntry();

        tipDecelerationAtiveTab = tab.add("Tip Deceleration", true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(5, 0)
            .withSize(1, 1)
            .getEntry();

        tipSmallStickAtiveTab = tab.add("Tip Small Stick", true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(5, 1)
            .withSize(1, 1)
            .getEntry();

        tipBigStickAtiveTab = tab.add("Tip Big Stick", true)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry();

        driveXTab = tab.add("Drive X", 0)
        .withPosition(0, 2)
        .withSize(1, 1)
        .getEntry();

        driveYTab = tab.add("Drive Y", 0)
        .withPosition(1, 2)
        .withSize(1, 1)
        .getEntry();

        rotateTab = tab.add("Rotate", 0)
        .withPosition(1, 3)
        .withSize(1, 1)
        .getEntry();

        odometryX = tab.add("Odometry X", 0)
        .withPosition(3, 2)
        .withSize(1, 1)
        .getEntry();

        odometryY = tab.add("Odometry Y", 0)
        .withPosition(4, 2)
        .withSize(1, 1)
        .getEntry();

        odometryDegrees = tab.add("Odometry Degrees", 0)
        .withPosition(2, 2)
        .withSize(1, 1)
        .getEntry();
      }
    }
  }

  public enum DriveMode {
    fieldCentric(0),
    frontCamCentric(1),
    leftCamCentric(2),
    rightCamCentric(3),
    limelightFieldCentric(4),
    killFieldCentric(5),
    sideKillFieldCentric(6);

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

  public void setDriveMode(DriveMode mode) {
    if (Constants.demo.inDemoMode) {
      if (mode != DriveMode.fieldCentric) {
        return;
      }
    }
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

  public void resetFieldCentric(int i) {

  }

  public void drive(double driveX, double driveY, double rotate) {
    if (Constants.driveEnabled) {
      double clock = runTime.get(); // cache value to reduce CPU usage
      double[] currentAngle = new double[4];
      for (int i = 0; i < swerveModule.length; i++) {
        currentAngle[i] = swerveModule[i].getInternalRotationDegrees();
      }

      Translation2d velocityXY = new VectorXY();
      Translation2d accelerationXY = new VectorXY();
      Translation2d driveXY = new VectorXY(driveX, driveY);

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
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModule) {
        module.setCoastmode();
      }
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModule) {
        module.setBrakeMode();
      }
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModule) {
        module.stop();
      }
    }
  }

  public void setModuleStates() {

  }

  public static double boundDegrees(double angleDegrees) {
    return 0;
  }

}


  // change vector --> translation2D since vector2d isn't library anymore


  // convert angle to range of +/- 180 degrees
  public static double boundDegrees(double angleDegrees) {
    double x = ((angleDegrees + 180) % 360) - 180;
    if (x < -180) {
      x += 360;
    }
    return x;
  }
}
