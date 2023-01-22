package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;;

public class ControlModule {
  public WheelPosition position;
  private AnalogPotentiometer rotationEncoder = null;
  private Encoder speedEncoder = null;

  protected double previousRate = 0;
  protected double previousTime = 0;
  protected double filteredAccel = 0;

  protected ControlModule(WheelPosition pos) {
    position = pos;
  }

  // *********************PID Helper Methods******************//

  public void resetSpeedModule() {
    if (speedEncoder != null)
      this.speedEncoder.reset();
  }

  // ************** Pos/Vel/Acc Helper Methods **************//

  public double getAngle() {
    if (rotationEncoder != null)
      return rotationEncoder.get();
    return -1.0;
  }

  public double getDistance() {
    if (speedEncoder != null)
      return speedEncoder.getDistance();
    return 0.0;
  }

  public double getVelocity() {
    if (speedEncoder != null)
      return speedEncoder.getRate();
    return 0.0;
  }

  public double snapshotAcceleration() {
    double currentRate = this.getVelocity();
    double currentTime = Timer.getFPGATimestamp();

    double acceleration = (currentRate - previousRate) / (currentTime - previousTime);

    previousRate = currentRate;
    previousTime = currentTime;

    filteredAccel = acceleration * 0.5 + filteredAccel * 0.5; // dampens random spikes due to the
                                                              // fact that we are deriving this
                                                              // value

    return filteredAccel;
  }

  public double getAcceleration() {
    return filteredAccel;
  }

  public Encoder getSpeedEncoder() {
    if (speedEncoder != null)
      return this.speedEncoder;
    return null;
  }

  public AnalogPotentiometer getRotationEncoder() {
    if (rotationEncoder != null)
      return this.rotationEncoder;
    return null;
  }

  public enum WheelPosition {
    FRONT_RIGHT(0), FRONT_LEFT(1), BACK_LEFT(2), BACK_RIGHT(3);

    public int wheelNumber;

    WheelPosition(int id) {
      wheelNumber = id;
    }

    public String toString() {
      switch (wheelNumber) {
        case 0:
          return "FRONT_RIGHT";
        case 1:
          return "FRONT_LEFT";
        case 2:
          return "BACK_LEFT";
        case 3:
          return "BACK_RIGHT";
        default:
          return "???";
      }
    }
  }
}
