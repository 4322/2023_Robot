package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.utility.OrangeMath;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.Manual;
import frc.robot.RobotContainer;

public class DriveManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;

  public DriveManual(Drive drivesubsystem) {
    drive = drivesubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Constants.joysticksEnabled && Constants.xboxEnabled) {

      // Joystick polarity:
      // Positive X is to the right
      // Positive Y is down
      // Positive Z is CW

      // Xbox polarity:
      // Positive X is to the right
      // Positive Y is down
      // (it's the same)

      // WPI uses a trigonometric coordinate system with the front of the robot
      // pointing toward positive X. Thus:
      // Positive X is forward
      // Positive Y is to the left
      // Positive angles are CCW
      // Angles have a range of +/- 180 degrees (need to verify this)

      // All variables in this program use WPI coordinates

      // Cache hardware status for consistency in logic and convert
      // joystick coordinates to WPI coordinates.

      // Cartesian inputs
      final double driveJoyRawX = -RobotContainer.driveStick.getY();
      final double driveJoyRawY = -RobotContainer.driveStick.getX();
      final double rotateJoyRawX = -RobotContainer.rotateStick.getY();
      final double rotateJoyRawY = -RobotContainer.rotateStick.getX();

      final double driveXboxRawX = -RobotContainer.xbox.getLeftY();
      final double driveXboxRawY = -RobotContainer.xbox.getLeftX();
      final double rotateXboxRawX = -RobotContainer.xbox.getRightY();
      final double rotateXboxRawY = -RobotContainer.xbox.getRightX();

      // Polar inputs
      final double driveJoyRawMag = OrangeMath.pythag(driveJoyRawX, driveJoyRawY);
      final double rotateJoyRawMag = OrangeMath.pythag(rotateJoyRawX, rotateJoyRawY);
      final double driveXboxRawMag = OrangeMath.pythag(driveXboxRawX, driveXboxRawY);
      final double rotateXboxRawMag = OrangeMath.pythag(rotateXboxRawX, rotateXboxRawY);

      final double driveJoyRawTheta = Math.atan2(driveJoyRawY, driveJoyRawX);
      final double rotateJoyRawTheta = Math.atan2(rotateJoyRawY, rotateJoyRawX);
      final double driveXboxRawTheta = Math.atan2(driveXboxRawY, driveXboxRawX);
      final double rotateXboxRawTheta = Math.atan2(rotateXboxRawY, rotateXboxRawX);

      // Joystick rotations
      final double driveJoyRawZ = -RobotContainer.driveStick.getZ();
      final double rotateJoyRawZ = -RobotContainer.rotateStick.getZ();

      // Deadbands and Active Checks
      final boolean driveJoyOutDeadband = Math.abs(driveJoyRawMag) > Manual.joystickDriveDeadband;
      final boolean rotateJoyOutDeadband = Math.abs(rotateJoyRawZ) > Manual.joystickRotateDeadband;
      final boolean driveXboxOutDeadband = Math.abs(driveXboxRawMag) > Manual.joystickDriveDeadband;
      final boolean rotateXboxOutDeadband =
          Math.abs(rotateXboxRawY) > Manual.joystickRotateDeadband;

      // Other variables
      double driveX;
      double driveY;
      double rotatePower;

      if (driveJoyOutDeadband && driveXboxOutDeadband) {
        driveX = (driveJoyRawX + driveXboxRawX) / 2;
        driveY = (driveJoyRawY + driveXboxRawY) / 2;
      } else if (driveJoyOutDeadband) {
        driveX = driveJoyRawX;
        driveY = driveJoyRawY;
      } else if (driveXboxOutDeadband) {
        driveX = driveXboxRawX;
        driveY = driveXboxRawY;
      } else {
        driveX = 0;
        driveY = 0;
      }

      if (rotateJoyOutDeadband && rotateXboxOutDeadband) {
        double combinedDeadband = Manual.joystickRotateDeadband + Manual.xboxRotateDeadband;
        rotatePower = (rotateJoyRawZ + rotateXboxRawY - combinedDeadband) / (2 - combinedDeadband);
      } else if (rotateJoyOutDeadband) {
        rotatePower =
            (rotateJoyRawZ - Manual.joystickRotateDeadband) / (1 - Manual.joystickRotateDeadband);
      } else if (rotateXboxOutDeadband) {
        rotatePower =
            (rotateXboxRawY - Manual.xboxRotateDeadband) / (1 - Manual.xboxRotateDeadband);
      } else {
        rotatePower = 0;
      }

      // Increase sensitivity
      driveX = driveX * driveX * driveX;
      driveY = driveY * driveY * driveY;
      rotatePower = Math.pow(rotatePower, 3);

      drive.drive(driveX, driveY, rotatePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
