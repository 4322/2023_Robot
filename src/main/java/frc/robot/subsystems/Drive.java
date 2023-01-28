package frc.robot.subsystems;

import java.util.ArrayList;
import java.lang.Object;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.SwerveModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;
import frc.utility.SnapshotTranslation2D;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private AHRS gyro;
  private PIDController rotPID;

  private Timer runTime = new Timer();

  private double latestVelocity;
  private double latestAcceleration;

  private ArrayList<SnapshotTranslation2D> velocityHistory = new ArrayList<SnapshotTranslation2D>();

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

  private RamseteController ram = new RamseteController();
  private SwerveDriveOdometry odometry;
  private double robotCentricOffsetDegrees;
  private boolean fieldRelative;

  private ShuffleboardTab tab;

  private GenericEntry rotErrorTab;
  private GenericEntry rotSpeedTab;
  private GenericEntry rotkP;
  private GenericEntry rotkD;
  private GenericEntry roll;
  private GenericEntry pitch;
  private GenericEntry botVelocityMag;
  private GenericEntry botAccelerationMag;
  private GenericEntry botVelocityAngle;
  private GenericEntry botAccelerationAngle;
  private GenericEntry tipDecelerationAtiveTab;
  private GenericEntry tipSmallStickAtiveTab;
  private GenericEntry tipBigStickAtiveTab;
  private GenericEntry driveXTab;
  private GenericEntry driveYTab;
  private GenericEntry rotateTab;
  private GenericEntry odometryX;
  private GenericEntry odometryY;
  private GenericEntry odometryDegrees;

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
        swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] =
            new SwerveModule(DriveConstants.frontRightRotationID, DriveConstants.frontRightDriveID,
                WheelPosition.FRONT_RIGHT, DriveConstants.frontRightEncoderID);
        swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] =
            new SwerveModule(DriveConstants.frontLeftRotationID, DriveConstants.frontLeftDriveID,
                WheelPosition.FRONT_LEFT, DriveConstants.frontLeftEncoderID);
        swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] =
            new SwerveModule(DriveConstants.rearRightRotationID, DriveConstants.rearRightDriveID,
                WheelPosition.BACK_RIGHT, DriveConstants.rearRightEncoderID);
        swerveModules[WheelPosition.BACK_LEFT.wheelNumber] =
            new SwerveModule(DriveConstants.rearLeftRotationID, DriveConstants.rearLeftDriveID,
                WheelPosition.BACK_LEFT, DriveConstants.rearLeftEncoderID);
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

        for (SwerveModule module : swerveModules) {
          module.init();
        }
      }

      ram.setTolerance(DriveConstants.poseError);

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Drivebase");

        rotErrorTab = tab.add("Rot Error", 0).withPosition(0, 0).withSize(1, 1).getEntry();

        rotSpeedTab = tab.add("Rotation Speed", 0).withPosition(0, 1).withSize(1, 1).getEntry();

        rotkP = tab.add("Rotation kP", DriveConstants.autoRotkP).withPosition(1, 0).withSize(1, 1)
            .getEntry();

        rotkD = tab.add("Rotation kD", DriveConstants.autoRotkD).withPosition(2, 0).withSize(1, 1)
            .getEntry();

        roll = tab.add("Roll", 0).withPosition(1, 1).withSize(1, 1).getEntry();

        pitch = tab.add("Pitch", 0).withPosition(2, 1).withSize(1, 1).getEntry();

        botVelocityMag = tab.add("Bot Vel Mag", 0).withPosition(3, 0).withSize(1, 1).getEntry();

        botAccelerationMag = tab.add("Bot Acc Mag", 0).withPosition(3, 1).withSize(1, 1).getEntry();

        botVelocityAngle = tab.add("Bot Vel Angle", 0).withPosition(4, 0).withSize(1, 1).getEntry();

        botAccelerationAngle =
            tab.add("Bot Acc Angle", 0).withPosition(4, 1).withSize(1, 1).getEntry();

        tipDecelerationAtiveTab = tab.add("Tip Deceleration", true)
            .withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 0).withSize(1, 1).getEntry();

        tipSmallStickAtiveTab = tab.add("Tip Small Stick", true)
            .withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 1).withSize(1, 1).getEntry();

        tipBigStickAtiveTab = tab.add("Tip Big Stick", true).withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(5, 2).withSize(1, 1).getEntry();

        driveXTab = tab.add("Drive X", 0).withPosition(0, 2).withSize(1, 1).getEntry();

        driveYTab = tab.add("Drive Y", 0).withPosition(1, 2).withSize(1, 1).getEntry();

        rotateTab = tab.add("Rotate", 0).withPosition(1, 3).withSize(1, 1).getEntry();

        odometryX = tab.add("Odometry X", 0).withPosition(3, 2).withSize(1, 1).getEntry();

        odometryY = tab.add("Odometry Y", 0).withPosition(4, 2).withSize(1, 1).getEntry();

        odometryDegrees =
            tab.add("Odometry Degrees", 0).withPosition(2, 2).withSize(1, 1).getEntry();
      }
    }
  }

  public enum DriveMode {
    fieldCentric(0), frontCamCentric(1), leftCamCentric(2), rightCamCentric(
        3), limelightFieldCentric(4), killFieldCentric(5), sideKillFieldCentric(6);

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
    if (gyro != null && gyro.isConnected() && !gyro.isCalibrated()) {
      return -gyro.getAngle();
    } else {
      return 0;
    }
  }

  @Override
  public void periodic() {
    if (Constants.driveEnabled) {
      // acceleration must be calculated once and only once per periodic interval
      for (SwerveModule module : swerveModules) {
        module.snapshotAcceleration();
      }

      updateOdometry();
    }

    if (Constants.debug) {  // don't combine if statements to avoid dead code warning
      if (Constants.gyroEnabled) {
        roll.setDouble(gyro.getRoll());
        pitch.setDouble(gyro.getPitch());
        odometryX.setDouble(getPose2d().getX());
        odometryY.setDouble(getPose2d().getY());
        odometryDegrees.setDouble(getPose2d().getRotation().getDegrees());
      }
    }
  }

  public void resetDisplacement() {
    if (Constants.gyroEnabled) {
      gyro.resetDisplacement();
    }
  }

  // rotation isn't considered to be movement
  public boolean isRobotMoving() {
    return latestVelocity >= DriveConstants.movingVelocityThresholdFtPerSec;
  }

  public void resetFieldCentric(double offset) {
    if (gyro != null) {
      gyro.setAngleAdjustment(0);
      gyro.setAngleAdjustment(-gyro.getAngle() + offset); 
    }
    setDriveMode(DriveMode.fieldCentric);
  }

  public void drive(double driveX, double driveY, double rotate) {
    if (Constants.driveEnabled) {
      double clock = runTime.get(); // cache value to reduce CPU usage
      double[] currentAngle = new double[4];
      for (int i = 0; i < swerveModules.length; i++) {
        currentAngle[i] = swerveModules[i].getInternalRotationDegrees();
      }

      Translation2d velocityXY = new Translation2d();
      Translation2d accelerationXY = new Translation2d();
      Translation2d driveXY = new Translation2d(driveX, driveY);

      // sum wheel velocity and acceleration vectors
      for (int i = 0; i < swerveModules.length; i++) {
        double wheelAngleDegrees = currentAngle[i];
        velocityXY.plus(new Translation2d(swerveModules[i].getVelocity(),
            Rotation2d.fromDegrees(wheelAngleDegrees)));
        accelerationXY.plus(new Translation2d(swerveModules[i].getAcceleration(),
            Rotation2d.fromDegrees(wheelAngleDegrees)));
      }
      latestVelocity = velocityXY.getNorm() / 4;
      latestAcceleration = accelerationXY.getNorm() / 4;
      velocityHistory
          .removeIf(n -> (n.getTime() < clock - DriveConstants.Tip.velocityHistorySeconds));
      velocityHistory.add(new SnapshotTranslation2D(velocityXY, clock));

      if (Constants.debug) {
        botVelocityMag.setDouble(latestVelocity);
        botAccelerationMag.setDouble(latestAcceleration);
        botVelocityAngle.setDouble(velocityXY.getAngle().getDegrees());
        botAccelerationAngle.setDouble(accelerationXY.getAngle().getDegrees());
        driveXTab.setDouble(driveX);
        driveYTab.setDouble(driveY);
        rotateTab.setDouble(rotate);
      }
    }
  }

  public boolean isAtTarget() {
    return ram.atReference();
  }

  // Uses a PID Controller to rotate the robot to a certain degree
  // Must be periodically updated to work
  public void driveAutoRotate(double driveX, double driveY, double rotateDeg, double toleranceDeg) {

    if (Constants.debug) {
      rotPID.setP(rotkP.getDouble(DriveConstants.autoRotkP));
      rotPID.setD(rotkD.getDouble(DriveConstants.autoRotkD));
    }

    double rotPIDSpeed = rotPID.calculate(0, rotateDeg);

    if (Math.abs(rotateDeg) <= toleranceDeg) {
      rotPIDSpeed = 0;
    } else if (Math.abs(rotPIDSpeed) < DriveConstants.minAutoRotateSpeed) {
      rotPIDSpeed = Math.copySign(DriveConstants.minAutoRotateSpeed, rotPIDSpeed);
    } else if (rotPIDSpeed > DriveConstants.maxAutoRotateSpeed) {
      rotPIDSpeed = DriveConstants.maxAutoRotateSpeed;
    } else if (rotPIDSpeed < -DriveConstants.maxAutoRotateSpeed) {
      rotPIDSpeed = -DriveConstants.maxAutoRotateSpeed;
    }

    drive(driveX, driveY, rotPIDSpeed);

    if (Constants.debug) {
      rotErrorTab.setDouble(rotateDeg);
      rotSpeedTab.setDouble(rotPIDSpeed);
    }
  }

  public void resetRotatePID() {
    rotPID.reset();
  }

  public void updateOdometry() {
    if (Constants.gyroEnabled) {
      odometry.update(
        gyro.getRotation2d(),
        // wheel locations must be in the same order as the WheelPosition enum values
        swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber].getState(),
        swerveModules[WheelPosition.FRONT_LEFT.wheelNumber].getState(),
        swerveModules[WheelPosition.BACK_LEFT.wheelNumber].getState(),
        swerveModules[WheelPosition.BACK_RIGHT.wheelNumber].getState()
      );
    }
  }

  public void resetodometry(Pose2d pose) {
    if (Constants.gyroEnabled) {
      odometry.resetPosition(pose, gyro.getRotation2d());
    }
  }

  public void setToFieldCentric() {
    fieldRelative = true;
    robotCentricOffsetDegrees = 0;
  }

  public void setToBotCentric(double offsetDeg) {
    fieldRelative = false;
    robotCentricOffsetDegrees = offsetDeg;
  }

  public void setCoastMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setCoastmode();
      }
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setBrakeMode();
      }
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
    }
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeedMetersPerSecond);
    int i = 0;
    for (SwerveModuleState s : states) {
      swerveModules[i].setDesiredState(s);
      i++;
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
