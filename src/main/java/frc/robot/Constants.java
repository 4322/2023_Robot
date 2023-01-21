package frc.robot;

public class Constants {
  public static final boolean clawEnabled = true;
  public static final boolean armEnabled = true;

  public static final class ClawConstants {// all temp values
    public static final int motorID = 1; // temp value
    public static final double rampRate = 0.8; // temp value
    public static final double IntakeVelocity = 300;
    public static final double EjectionVelocity = -200;

    public static enum IntakeMode {
      ejecting, stationary, intaking
    };
  }

  public static final class ArmConstants {
    public static final int motorID = 0;
    public static final double rampRate = 0.0;
    public static final double forward = 1;
    public static final double backward = -1;
  }
}
