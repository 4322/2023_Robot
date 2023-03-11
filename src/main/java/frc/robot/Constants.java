package frc.robot;

import frc.robot.subsystems.SwerveDrive.ControlModule.WheelPosition;
import frc.utility.OrangeMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
  public static final boolean debug = false;
  public static final boolean inDemoMode = false;
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
  public static final boolean armSensorEnabled = true;
  public static final boolean clawEnabled = true;
  public static final boolean driveEnabled = true;
  public static final boolean gyroEnabled = true;
  public static final boolean joysticksEnabled = true;
  public static final boolean xboxEnabled = true;
  public static final boolean limeLightEnabled = true;
  public static final boolean ledEnabled = true;
  public static final boolean colorSensorEnabled = false;

  // To tune a NEO with the REV Hardware Client, the motor must be initialized
  // in the application to enable it and no set() commands can be issued because
  // the REV library will continuously send the same command, thereby overriding
  // tuning commands from the REV Hardware CLient.
  public static final boolean armTuningMode = false;
  public static final boolean clawTuningMode = false;

  public static final int falconEncoderUnits = 2048;
  public static final double inchesToMeters = 0.0254;
  public static final double feetToMeters = inchesToMeters * 12;
  public static final int fastStatusPeriodBaseMs = 13;
  public static final int shuffleboardStatusPeriodBaseMs = 75;
  public static final int slowStatusPeriodBaseMs = 180;
  public static final int verySlowStatusPeriodSparkBaseMs = 1000;
  public static final int fastStatusPeriodMaxMs = 18;
  public static final int shuffleboardStatusPeriodMaxMs = 90;  // for interactive response
  public static final int slowStatusPeriodMaxMs = 255;
  public static final int controllerConfigTimeoutMs = 50;

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

    public static final double distWheelMetersX = 0.62865; // 24.75 in
    public static final double distWheelMetersY = 0.62865; // 24.75 in

    // Max speed is 200000 ticks / 1 s

    public static final double maxSpeedMetersPerSecond = 10 * OrangeMath.falconEncoderToMeters(20000,
        OrangeMath.inchesToMeters(OrangeMath.getCircumference(Drive.wheelDiameterInches)),
        Drive.gearRatio);
    public static final double maxRotationSpeedRadSecond = 12.2718;

    public static final double movingVelocityThresholdFtPerSec = 0.2;

    public static final double minAutoRotateSpeed = 0.03;
    public static final double maxAutoRotateSpeed = Constants.demo.inDemoMode? Constants.demo.rotationScaleFactor : 0.5;

    public static final double drivePolarDeadband = 0.06;
    public static final double rotatePolarDeadband = 0.5;
    public static final double twistDeadband = 0.08;

    public static final double manualRotateToleranceDegrees = 1.5;

    // Values for auto balance
    public static final double autoBalanceFlatPower = 0.25;
    public static final double autoBalanceRampPower = 0.1; // slower speed when approaching midpoint of charging station
    public static final double chargeStationTiltedMinDeg = 7.0;
    public static final double chargeStationBalancedMaxDeg = 5.0;
    public static final double autoDriveOverChargeFlatMaxDeg = 3.0;
    public static final double autoDriveOverChargeFlatSec = 0.5;
    public static final double autoBalanceTimeoutSec = 4.0;
    public static final double autoDriveOverChargeTimeoutSec = 6.0;

    // 1 degree
    public static final Pose2d poseError =
        new Pose2d(new Translation2d(0.1, 0.1), new Rotation2d(0.0174533));

    public static final double disableBreakSec = 2.0;

    public static final class Manual {

      public static final double joystickDriveDeadband = 0.1;
      public static final double joystickRotateLeftDeadband = 0.4;  // both joysticks have a huge left twist deadzone
      public static final double joystickRotateRightDeadband = 0.2;

      public static final double xboxDriveDeadband = 0.1;
      public static final double xboxRotateDeadband = 0.2;
      public static final double rotateToleranceDegrees = 1.5;

    }

    public static final class Auto {

      // Max acceleration is 180000 ticks / s^2

      // Values for autonomous path finding
      public static final double autoMaxSpeedMetersPerSecond = 0.75 * DriveConstants.maxSpeedMetersPerSecond;
      public static final double autoMaxAccelerationMetersPerSec2 = 0.75 * OrangeMath.falconEncoderToMeters(180000,
          OrangeMath.inchesToMeters(OrangeMath.getCircumference(Drive.wheelDiameterInches)),
          Drive.gearRatio);

      public static final double autoRotkP = 0.005;
      public static final double autoRotkD = 0.0002;
      public static final double minAutoRotateSpeed = 0.0;  // 0.03
      public static final double maxAutoRotateSpeed = 0.5;

    }

    public static final class Tip {

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
        CANCoderOffsetDegrees[WheelPosition.FRONT_LEFT.wheelNumber] = 2.637 - 90;
        CANCoderOffsetDegrees[WheelPosition.BACK_RIGHT.wheelNumber] = 22.939 - 90;
        CANCoderOffsetDegrees[WheelPosition.BACK_LEFT.wheelNumber] = -72.773;
      }
    }

    public static final class Drive {

      public static final double configClosedLoopRamp = 0;

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

      public static final double wheelDiameterInches = 3.95;
      public static final double gearRatio = 7.8;
      public static final double kP = 0.05;
      public static final double kI = 0.0002;
      public static final double kD = 0.0;
      public static final double kIz = 500;
      public static final double kFF = 0.054;
      
    }

    public static final class Trajectory {

      public static final class PIDXY {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }

      public static final class PIDR {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }
      
    }
  }

  public static final class ClawConstants {// all temp values
    public static final int motorID = 16; // temp value
    public static final double rampRate = 0.8; // temp value


    public static final double intakePower = 0.4; // don't exceed 0.6 if you don't want to smoke the motor!
    public static final double outtakePower = -0.4; 
    public static final double stallIntakePower = 0.07; // don't exceed 0.07 if you don't want to smoke the motor!
    public static final double stallOuttakePower = -0.06;

    public static final double stallTime = 0.2; // 200 ms
    public static final double stallRPMLimit = 1000;

    public static enum ClawMode {
      ejecting, stationary, intaking
    }
  }

  public static final class ArmConstants {
    public static final int leftMotorID = 15;
    public static final int rightMotorID = 14;
    public static final double rampRate = 0.3; // good range: 0.3 to 0.5
    public static final double logIntervalSeconds = 0.5;
  
    public static final int maxPosition = 72;
    public static final int minPosition = 0;

    public static final double manualDeadband = 0;

    public static final double kMaxRange = 0;

    public static final double LoadPosition = 2;
    public static final double LoadHighPosition = 10;
    public static final double MidScoringPosition = 68;
    public static final double HighScoringPosition = 60;
    
    public static final double ArmHomingPower = -0.1;
    public static double homingTimeout = 3; // seconds

    public static final double positionToleranceInternal = 0.3;

    public static final class SmartMotion { // SmartMotion values need to be checked (not k values), currently not using
      public static final double kP = 0.1;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kMaxOutput = 0.5;
      public static final double kMinOutput = -0.5;
      public static final double minVel = 0;
      public static final double maxVel = 3000; // rpm
      public static final double maxAcc = 10000;
      public static final double positionTolerance = 0.2;
    }
  }

  public static final class LEDConstants
  {
    public static final int pcmID=30;
    public static final int rPort1=2;
    public static final int gPort1=1;
    public static final int bPort1=3;
    public static final int rPort2=6;
    public static final int gPort2=5;
    public static final int bPort2=7;
    public static final int pPort1=0;
    public static final int pPort2=4;
    
  }

  public static final class LimelightConstants {
    public static final Integer[] tapePipelines = {0};
    public static final Integer[] tagPipelines = {1};

    public static final double limelightAngle = 0;
    public static final double limelightHeight = 26.125;

    // Tape heights are 1 inch higher than described in manual to account for
    // height to center of tape
    public static final double middleTapeHeight = 23.125;
    public static final double highTapeHeight = 42.875;

    // AprilTag heights are 4 inches higher than described in manual to account
    // for height to center of tag
    public static final double gridAprilTagHeight = 18.25;
    public static final double doubleSubstationAprilTagHeight = 27.375;

    // Threshold for limelight target height
    // above = high tape, below = middle tape
    public static final double targetHeightThresholdDeg = 0;
  }
}
