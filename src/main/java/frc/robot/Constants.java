package frc.robot;

public class Constants {
  public static final boolean armEnabled = false;
  public static final boolean clawEnabled = false;
  public static final boolean driveEnabled = false;

  public static final class DriveConstants{
    public static final int frontRightDriveID = 0;
    public static final int frontRightRotationID = 0;
    public static final int rearRightDriveID = 0;
    public static final int rearRightRotationID = 0;
    public static final int frontLeftDriveID = 0;
    public static final int frontLeftRotationID = 0;
    public static final int rearLeftDriveID = 0;
    public static final int rearLeftRotationID = 0;

    public static final int frontRightEncoder = 0;
    public static final int rearRightEncoder = 0;
    public static final int frontLeftEncoder = 0;
    public static final int rearLeftEncoder = 0;

    public static final int encoderResolution = 0;

    public static final double distWheelMetersX = 0;
    public static final double disWheelMetersY = 0;

    public static final double wheelBaseLengthFeet = 0;
    public static final double wheelBaseWidthFeet = 0;

    public static final double maxSpeedMetersSecond = 0;
    public static final double maxRotationSpeedRadSecond = 0;

    public static final class Rotation {
      
    }

    public static final class Drive {

    }
  }

  public static final class ClawConstants {
    public static final int motorID = 1; //temp value
    public static final double rampRate = 0.8; //temp value
  }
  
  public static final class ArmConstants {
      public static final int motorID = 0;
      public static final double rampRate = 0.0;
      public static final double forward = 1;
      public static final double backward = -1;
  }
}
