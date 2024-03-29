package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.SwerveModule;
import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;
import frc.utility.OrangeMath;
import frc.utility.SnapshotTranslation2D;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private AHRS gyro;
  private PIDController rotPID;

  private Timer runTime = new Timer();

  private double latestVelocity;
  private double latestAcceleration;
  private double pitchOffset;

  private ArrayList<SnapshotTranslation2D> velocityHistory = new ArrayList<SnapshotTranslation2D>();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.frontRightWheelLocation, 
  DriveConstants.frontLeftWheelLocation, DriveConstants.backLeftWheelLocation, DriveConstants.backRightWheelLocation);

  private SwerveDriveOdometry odometry;
  private ShuffleboardTab tab;

  private GenericEntry rotErrorTab;
  private GenericEntry rotSpeedTab;
  private GenericEntry rotkP;
  private GenericEntry rotkD;
  private GenericEntry yawTab;
  private GenericEntry rollTab;
  private GenericEntry pitchTab;
  private GenericEntry botVelocityMag;
  private GenericEntry botAccelerationMag;
  private GenericEntry botVelocityAngle;
  private GenericEntry botAccelerationAngle;
  private GenericEntry driveXTab;
  private GenericEntry driveYTab;
  private GenericEntry rotateTab;
  private GenericEntry odometryX;
  private GenericEntry odometryY;
  private GenericEntry odometryDegrees;
  private GenericEntry angularVel;

  public Drive() {
    runTime.start();
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
    }
  }

  public void init() {
    if (Constants.driveEnabled) {
      rotPID = new PIDController(DriveConstants.Auto.autoRotkP, 0, DriveConstants.Auto.autoRotkD);

      for (SwerveModule module : swerveModules) {
        module.init();
      }

      if (Constants.gyroEnabled) {
        gyro = new AHRS(SPI.Port.kMXP, (byte) 66);
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getModulePostitions());

        // wait for first gyro reading to be received
        try {
          Thread.sleep(250);
        } catch (InterruptedException e) {
        }

        resetFieldCentric(0);
      } 

      if (Constants.debug) {
        tab = Shuffleboard.getTab("Drivebase");

        rotErrorTab = tab.add("Rot Error", 0).withPosition(0, 0).withSize(1, 1).getEntry();

        rotSpeedTab = tab.add("Rotation Speed", 0).withPosition(0, 1).withSize(1, 1).getEntry();

        rotkP = tab.add("Rotation kP", DriveConstants.Auto.autoRotkP).withPosition(1, 0).withSize(1, 1)
            .getEntry();

        rotkD = tab.add("Rotation kD", DriveConstants.Auto.autoRotkD).withPosition(2, 0).withSize(1, 1)
            .getEntry();

        yawTab = tab.add("Yaw", 0).withPosition(0, 3).withSize(1, 1).getEntry();

        rollTab = tab.add("Roll", 0).withPosition(1, 1).withSize(1, 1).getEntry();

        pitchTab = tab.add("Pitch", 0).withPosition(2, 1).withSize(1, 1).getEntry();

        botVelocityMag = tab.add("Bot Vel Mag", 0).withPosition(3, 0).withSize(1, 1).getEntry();

        botAccelerationMag = tab.add("Bot Acc Mag", 0).withPosition(3, 1).withSize(1, 1).getEntry();

        botVelocityAngle = tab.add("Bot Vel Angle", 0).withPosition(4, 0).withSize(1, 1).getEntry();

        botAccelerationAngle = tab.add("Bot Acc Angle", 0).withPosition(4, 1).withSize(1, 1).getEntry();

        angularVel = tab.add("Angular Vel", 0).withPosition(5, 0).withSize(1, 1).getEntry();

        driveXTab = tab.add("Drive X", 0).withPosition(0, 2).withSize(1, 1).getEntry();

        driveYTab = tab.add("Drive Y", 0).withPosition(1, 2).withSize(1, 1).getEntry();

        rotateTab = tab.add("Rotate", 0).withPosition(1, 3).withSize(1, 1).getEntry();

        odometryX = tab.add("Odometry X", 0).withPosition(3, 2).withSize(1, 1).getEntry();

        odometryY = tab.add("Odometry Y", 0).withPosition(4, 2).withSize(1, 1).getEntry();

        odometryDegrees = tab.add("Odometry Degrees", 0).withPosition(2, 2).withSize(1, 1).getEntry();
      }
    }
  }

  public enum DriveMode {
    fieldCentric(0), botCentric(1);

    private int value;

    DriveMode(int value) {
      this.value = value;
    }

    public int get() {
      return value;
    }
  }

  private static DriveMode driveMode = DriveMode.fieldCentric;

  public static DriveMode getDriveMode() {
    if (Constants.driveEnabled) {
      DataLogManager.log("DriveMode is " + driveMode);
      return driveMode;
    } else {
      return null;
    }
  }

  public void setDriveMode(DriveMode mode) {
    if (Constants.driveEnabled) {
    // driveMode = mode;
    // switch (mode) {
    //   case fieldCentric:
    //   case botCentric:
    // }
    // DataLogManager.log("DriveMode set to " + mode);
      return;
    }
  }

  // get the yaw angle
  public double getAngle() { 
    if (gyro != null && gyro.isConnected() && !gyro.isCalibrating() && Constants.gyroEnabled) {
        return -gyro.getAngle();
    } else {
      return 0;
    }
  }

  // Get pitch in degrees. Positive angle is the front of the robot raised.
  public double getPitch() {
    if (gyro != null && gyro.isConnected() && !gyro.isCalibrating() && Constants.gyroEnabled) {
      return gyro.getPitch() - pitchOffset;
    } else {
      return 0;
    }
  }

  // get the change of robot heading in degrees per sec
  public double getAngularVelocity() {
    if (gyro != null && gyro.isConnected() && !gyro.isCalibrating() && Constants.gyroEnabled) {
      return -gyro.getRate();
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
      if (Constants.gyroEnabled) {
        updateOdometry();
        if (Constants.debug) { // don't combine if statements to avoid dead code warning
          yawTab.setDouble(getAngle());
          rollTab.setDouble(gyro.getRoll());
          pitchTab.setDouble(getPitch());
          odometryX.setDouble(getPose2d().getX());
          odometryY.setDouble(getPose2d().getY());
          odometryDegrees.setDouble(getPose2d().getRotation().getDegrees());
          angularVel.setDouble(getAngularVelocity());
        }
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
    if (Constants.driveEnabled) {
      return latestVelocity >= DriveConstants.movingVelocityThresholdFtPerSec;
    } else {
      return false;
    }
  }

  public void resetFieldCentric(double offset) {
    if (Constants.driveEnabled && Constants.gyroEnabled) {
      if (gyro != null) {
        gyro.setAngleAdjustment(0);
        gyro.setAngleAdjustment(-gyro.getAngle() + offset);
        pitchOffset = gyro.getPitch();
      }
      setDriveMode(DriveMode.fieldCentric);
    }
  }
  //this drive function is for regular driving when pivot point is at robot center point
  public void drive(double driveX, double driveY, double rotate) {
    drive(driveX, driveY, rotate, new Translation2d());
  }

  // main drive function accounts for spinout when center point is on swerve modules
  public void drive(double driveX, double driveY, double rotate, Translation2d centerOfRotation) {
    if (Constants.driveEnabled && Constants.gyroEnabled) {
      double clock = runTime.get(); // cache value to reduce CPU usage
      double[] currentAngle = new double[4];
      for (int i = 0; i < swerveModules.length; i++) {
        currentAngle[i] = swerveModules[i].getInternalRotationDegrees();
      }

      Translation2d velocityXY = new Translation2d();
      Translation2d accelerationXY = new Translation2d();
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
      // convert to proper units
      rotate = rotate * DriveConstants.maxRotationSpeedRadSecond;
      driveX = driveX * DriveConstants.maxSpeedMetersPerSecond;
      driveY = driveY * DriveConstants.maxSpeedMetersPerSecond;

      // ready to drive!
      if ((driveX == 0) && (driveY == 0) && (rotate == 0)) {
        stop();
      } else {
        Rotation2d robotAngle;
          robotAngle = gyro.getRotation2d();

        // create SwerveModuleStates inversely from the kinematics

        var swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, robotAngle), centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
            Constants.DriveConstants.maxSpeedMetersPerSecond);
        for (int i = 0; i < swerveModules.length; i++) {
          swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
      }
    }
  }

  // Rotate the robot to a specific heading while driving.
  // Must be invoked periodically to reach the desired heading.
  public void driveAutoRotate(double driveX, double driveY, double targetDeg, double toleranceDeg) {
    if (Constants.driveEnabled) {
 
      if (Constants.debug) {
        rotPID.setP(rotkP.getDouble(DriveConstants.Auto.autoRotkP));
        rotPID.setD(rotkD.getDouble(DriveConstants.Auto.autoRotkD));
      }

      // Don't use absolute heading for PID controller to avoid discontinuity at +/- 180 degrees
      double headingChangeDeg = OrangeMath.boundDegrees(targetDeg - getAngle());
      double rotPIDSpeed = rotPID.calculate(0, headingChangeDeg);

      if (Math.abs(headingChangeDeg) <= toleranceDeg) {
        rotPIDSpeed = 0;  // don't wiggle
      } else if (Math.abs(rotPIDSpeed) < DriveConstants.Auto.minAutoRotatePower) {
        rotPIDSpeed = Math.copySign(DriveConstants.Auto.minAutoRotatePower, rotPIDSpeed);
      } else if (rotPIDSpeed > DriveConstants.Auto.maxAutoRotatePower) {
        rotPIDSpeed = DriveConstants.Auto.maxAutoRotatePower;
      } else if (rotPIDSpeed < -DriveConstants.Auto.maxAutoRotatePower) {
        rotPIDSpeed = -DriveConstants.Auto.maxAutoRotatePower;
      }

      drive(driveX, driveY, rotPIDSpeed);

      if (Constants.debug) {
        rotErrorTab.setDouble(headingChangeDeg);
        rotSpeedTab.setDouble(rotPIDSpeed);
      }
    }
  }

  public void resetRotatePID() {
    if (Constants.driveEnabled) {
      rotPID.reset();
    }
  }

  public void updateOdometry() {
    if (Constants.gyroEnabled) {
      odometry.update(gyro.getRotation2d(), getModulePostitions());
    }
  }

  public void resetOdometry(Pose2d pose) {
    if (Constants.gyroEnabled) {
      odometry.resetPosition(gyro.getRotation2d(), getModulePostitions(), pose);
    }
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
    if (Constants.driveEnabled) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeedMetersPerSecond);
      int i = 0;
      for (SwerveModuleState s : states) {
        swerveModules[i].setDesiredState(s);
        i++;
      }
    }
  }

  public SwerveModulePosition[] getModulePostitions() {
    if (Constants.driveEnabled) {
      // wheel locations must be in the same order as the WheelPosition enum values
      return new SwerveModulePosition[] {
          swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.FRONT_LEFT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.BACK_LEFT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.BACK_RIGHT.wheelNumber].getPosition() };
    } else {
      return null;
    }
  }

  public SwerveDriveKinematics getKinematics() {
    if (Constants.driveEnabled) {
      return kinematics;
    } else {
      return null;
    }
  }
}
