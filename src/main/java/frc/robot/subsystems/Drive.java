package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive.SwerveModule;

public class Drive extends SubsystemBase{

  private SwerveModule[] swerveModule = new SwerveModule[4];

  public Drive() {

  }

  public void init() {

  }

  public enum DriveMode {
    fieldCentric(0);

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

  public void setDriveMode() {

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

  public void resetFieldCentric() {

  }

  public void drive() {

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

  }

  public void setBrakemMode() {

  }

  public void stop() {

  }

  public void setModuleStates() {

  }
  
  public static double boundDegrees() {
    return 0;
  }
}
