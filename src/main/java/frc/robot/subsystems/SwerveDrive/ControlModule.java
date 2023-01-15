package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;

public class ControlModule {

  private Encoder speedEncoder = null;
  protected ControlModule() {

  }

  //*********************PID Helper Methods******************//

  public void resetSpeedmodule() {

  }

  //************** Pos/Vel/Acc Helper Methods **************//

  public double getAngle() {
    return 0;
  }

  public double getDistance() {
    return 0;
  }

  public double getVelocity() {
    return 0;
  }

  public double snapshotAcceleration() {
    return 0;
  }

  public double getAcceleration() {
    return 0;
  }

  public Encoder getSpeedEncoder() {
    return null;
  }

  public AnalogPotentiometer getRotationEncoder() {
    return null;
  }

  public enum WheelPosition{
    
  }
}
