package frc.robot;

public class Constants {

public static final boolean debug = false;

  public static final boolean armEnabled = true;
  public static final boolean clawEnabled = true;

  public static final int falconEncoderUnits = 2048;

  public static final class ArmConstants {
    public static final int motorID = 0;
    public static final double rampRate = 0.0;
    public static final double forward = 1;
    public static final double backward = -1;
    public static final int positionTolerance = 100;
    public static final double logIntervalSeconds = 1;
    public static final int maxPosition = 1000;
    public static final int minPosition = -1000;
  }
    
  public static final class ClawConstants {
    public static final int motorID = 1; //temp value
    public static final double rampRate = 0.8; //temp value
  }
}
