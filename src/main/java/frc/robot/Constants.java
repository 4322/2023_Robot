package frc.robot;

import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
  public static final boolean debug = false;

  public static final class demo {
    public enum DriveMode {
      OFF, SLOW_ROTATE_ONLY, SLOW_DRIVE
    }

    public static final boolean inDemoMode = false;
    public static final DriveMode driveMode = DriveMode.SLOW_DRIVE;

    public static final double driveScaleFactor = 0.15;
    public static final double rotationScaleFactor = 0.1;
  }

  public static final boolean armEnabled = true;
  public static final boolean armSensorEnabled = false;
  public static final boolean clawEnabled = true;
  public static final boolean driveEnabled = true;
  public static final boolean gyroEnabled = true;
  public static final boolean joysticksEnabled = true;
  public static final boolean limeLightEnabled = true;
  public static final boolean ledEnabled = true;
  public static final boolean colorSensorEnabled = false;

  public static final int falconEncoderUnits = 2048;
  public static final double inchesToMeters = 0.0254;
  public static final double feetToMeters = inchesToMeters * 12;
  public static final int slowStatusPeriodBaseMs = 180;
  public static final int fastStatusPeriodBaseMs = 13;
  public static final int fastStatusPeriodMaxMs = 18;
  public static final int slowStatusPeriodMaxMs = 255;
  public static final int controllerConfigTimeoutMs = 50;
  public static final int verySlowStatusPeriodSparkBaseMs = 1000;

  public static final class DriveConstants {
    public static final int frontRightDriveID = 3;
    public static final int frontRightRotationID = 5;
    public static final int rearRightDriveID = 6;
    public static final int rearRightRotationID = 7;
    public static final int frontLeftDriveID = 2;
    public static final int frontLeftRotationID = 4;
    public static final int rearLeftDriveID = 8;
    public static final int rearLeftRotationID = 9;
    
    public static final int frontRightEncoderID = 10;
    public static final int rearRightEncoderID = 12;
    public static final int frontLeftEncoderID = 11;
    public static final int rearLeftEncoderID = 13;

    public static final int encoderResolution = 2048;

    public static final double distWheelMetersX = 0.314325; // 24.75 in
    public static final double distWheelMetersY = 0.314325; // 24.75 in

    public static final double maxSpeedMetersPerSecond = 3.6576;
    public static final double maxRotationSpeedRadSecond = 12.2718;

    public static final double autoRotkP = 0.008;
    public static final double autoRotkD = 0.0004;

    public static final double movingVelocityThresholdFtPerSec = 0.2;

    public static final double minAutoRotateSpeed = 0.03;
    public static final double maxAutoRotateSpeed = Constants.demo.inDemoMode? Constants.demo.rotationScaleFactor : 0.5;

    public static final double drivePolarDeadband = 0.06;
    public static final double rotatePolarDeadband = 0.5;
    public static final double twistDeadband = 0.08;

    public static final double manualRotateToleranceDegrees = 1.5;

    // Values for autonomous path finding
    public static final double autoMaxSpeedMetersPerSecond = 3.5;
    public static final double autoMaxAccelerationMetersPerSec2 = 2.5;
    public static final double autoMaxRotationRadPerSecond = Math.PI * 2;
    public static final double autoMaxRotationAccelerationRadPerSec2 = Math.PI * 4;

    // 1 degree
    public static final Pose2d poseError =
        new Pose2d(new Translation2d(0.1, 0.1), new Rotation2d(0.0174533));
    public static final double disableBreakSec = 2.0;

    public final static class Tip {
      public static final double highVelocityFtPerSec = 6.0;
      public static final double lowVelocityFtPerSec = 3.0;
      public static final double highAccFtPerSec2 = 8.0;
      public static final double lowAccFtPerSec2 = 4.0;
      public static final double velAccDiffMaxDeg = 30;
      public static final double highPowerOff = 0.4;
      public static final double lowPowerOff = 0.19;
      public static final double highSpeedSteeringChangeMaxDegrees = 20;
      public static final double velocityHistorySeconds = 0.1;
    }

    public static final class Rotation {

      public static final double minAutoRotateSpeed = 0.0;
      public static final double maxAutoRotateSpeed = 0.0;

      public static final double movingVelocityThresholdFtPerSec = 0.2;

      public static final Pose2d poseError =
          new Pose2d(new Translation2d(0.1, 0.1), new Rotation2d(0.0174533));

      public static final double kP = 1.2;
      public static final double kD = 6.0;

      public static final double configCLosedLoopRamp = 0.08;
      public static final double minPower = 0.0; // allow for tighter tolerance
      public static final double maxPower = 0.3; // reduce gear wear and overshoot
      public static final double countToDegrees = 360.0 / encoderResolution * 12 / 24 * 14 / 72;

      public static final double configVoltageCompSaturation = 11.5;
      public static final boolean enableVoltageCompensation = true;

      public static final boolean statorEnabled = true;
      public static final double statorLimit = 40;
      public static final double statorThreshold = 45;
      public static final double statorTime = 1.0;

      public static final boolean supplyEnabled = true;
      public static final double supplyLimit = 30;
      public static final double supplyThreshold = 35;
      public static final double supplyTime = 0.5;

      public static final double allowableClosedloopError = 0.35 / countToDegrees;

      // values obtained from swerve module zeroing procedure
      // positive angles are CCW rotation from forward
      public static final double[] CANCoderOffsetDegrees;
      static {
        CANCoderOffsetDegrees = new double[4];
        CANCoderOffsetDegrees[WheelPosition.FRONT_RIGHT.wheelNumber] = 149.941;
        CANCoderOffsetDegrees[WheelPosition.FRONT_LEFT.wheelNumber] = 2.637 + 90;
        CANCoderOffsetDegrees[WheelPosition.BACK_RIGHT.wheelNumber] = 22.939 + 90;
        CANCoderOffsetDegrees[WheelPosition.BACK_LEFT.wheelNumber] = -72.773;
      }
    }

    public static final class Drive {

      // TODO: Needs tuning
      public static final double configClosedLoopRamp = 0.08;

      public static final double voltageCompSaturation = 11.5;
      public static final boolean enableVoltageCompensation = true;

      public static final double brakeModeDeadband = 0.01;

      public static final boolean statorEnabled = true;
      public static final double statorLimit = 40;
      public static final double statorThreshold = 45;
      public static final double statorTime = 1.0;

      public static final boolean supplyEnabled = true;
      public static final double supplyLimit = 40;
      public static final double supplyThreshold = 45;
      public static final double supplyTime = 0.5;

      public static final double wheelDiameterInches = 4.0;
      public static final double gearRatio = 6.55;
      public static final double kP = 0.045;
      public static final double kI = 0.000065;
      public static final double kD = 0.0;
      public static final double kIz = 500;
      public static final double kFF = 0.05;
    }

    public static final class Trajectory {
      public static final class PIDXY {
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
      }

      public static final class PIDR {
        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0.5;
      }
    }

  }

  public static final class ClawConstants {// all temp values
    public static final int motorID = 16; // temp value
    public static final double rampRate = 0.8; // temp value
    public static final double IntakeVelocity = 0.5;
    public static final double EjectionVelocity = -0.2;

    public static enum ClawMode {
      ejecting, stationary, intaking
    };
  }

  public static final class ArmConstants {
    public static final int leftMotorID = 15;
    public static final int rightMotorID = 14;
    public static final double rampRate = 0.0;
    public static final double forward = 1;
    public static final double backward = -1;
    public static final double logIntervalSeconds = 0.5;
    public static final int positionTolerance = 100;
  
    public static final int maxPosition = 1000;
    public static final int minPosition = -1000;
    public static final double manualDeadband = 0;

    public static final double kMaxRange = 0;

    public static final double LoadPosition = 0.0;
    public static final double MidScoringPosition = 1000.0;
    public static final double HighScoringPosition = 2000.0;
  }
  public static final class LEDConstants
  {
    public static final int pcmID=30;
    public static final int rPort1=3;
    public static final int gPort1=1;
    public static final int bPort1=2;
    public static final int rPort2=7;
    public static final int gPort2=5;
    public static final int bPort2=6;
    public static final int pPort1=0;
    public static final int pPort2=4;
    
  }
  public static final class LimelightConstants {
    // All distance values in inches
    public static final double limelightAngle = 0;
    public static final double targetHeight = 0;
    public static final double limelightHeight = 0;
  }
}
