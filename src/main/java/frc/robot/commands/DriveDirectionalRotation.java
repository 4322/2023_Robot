package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.utility.OrangeMath;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.Manual;
import frc.robot.Constants.DriveConstants.Auto.rotationDir;
import frc.robot.Constants.DriveConstants.Auto;
import frc.robot.RobotContainer;

public class DriveDirectionalRotation extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final rotationDir rotation;
  private double rotationDeg;
  private Timer timeAtPos = new Timer();
  private Timer timeout = new Timer();

  public DriveDirectionalRotation(Drive drivesubsystem, rotationDir rotationDirection) {
    drive = drivesubsystem;
    rotation = rotationDirection;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch(rotation) {
      case forward:
        rotationDeg = 0;
        break;
      case left:
        rotationDeg = 90;
        break;
      case backward:
        rotationDeg = 180;
        break;
      case right:
        rotationDeg = 270;
        break;
    }

    timeAtPos.reset();
    timeout.reset();
    timeout.start();
  }

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
      // All "theta" variables are in radians

      // Dual driver inputs need to be processed in an additive manner
      // instead of being averaged to avoid discontinuities.

      // Cache hardware status for consistency in logic and convert
      // joystick/Xbox coordinates to WPI coordinates.
      final double drive1RawX = -RobotContainer.driveStick.getY();
      final double drive1RawY = -RobotContainer.driveStick.getX();

      final double drive2RawX = -RobotContainer.xbox.getLeftY();
      final double drive2RawY = -RobotContainer.xbox.getLeftX();

      // Deadbands are dependent on the type of input device
      final double drive1Deadband = Manual.joystickDriveDeadband;
      final double drive2Deadband = Manual.xboxDriveDeadband;

      // Convert raw drive inputs to polar coordinates for more precise deadband correction
      final double drive1RawMag = OrangeMath.pythag(drive1RawX, drive1RawY);
      final double drive2RawMag = OrangeMath.pythag(drive2RawX, drive2RawY);
      final double drive1RawTheta = Math.atan2(drive1RawY, drive1RawX);
      final double drive2RawTheta = Math.atan2(drive2RawY, drive2RawX);

      // Normalize the drive inputs over deadband in polar coordinates.
      // Process each set of inputs separately to avoid a discontinuity  
      // when the second input suddenly exceeds deadband.
      double drive1Mag = 0;
      if (drive1RawMag > drive1Deadband) {
        drive1Mag = (drive1RawMag - drive1Deadband) / (1 - drive1Deadband);
        drive1Mag = drive1Mag * drive1Mag * drive1Mag;  // Increase sensitivity efficiently
      } 
      double drive2Mag = 0;
      if (drive2RawMag > drive2Deadband) {
        drive2Mag = (drive2RawMag - drive2Deadband) / (1 - drive2Deadband);
        drive2Mag = drive2Mag * drive2Mag * drive2Mag;  // Increase sensitivity efficiently
      } 
      // Convert back to cartesian coordinates for proper vector addition
      double drive1X = Math.cos(drive1RawTheta) * drive1Mag;
      double drive1Y = Math.sin(drive1RawTheta) * drive1Mag;
      double drive2X = Math.cos(drive2RawTheta) * drive2Mag;
      double drive2Y = Math.sin(drive2RawTheta) * drive2Mag;

      // Add the drive vectors
      double driveX = drive1X + drive2X;
      double driveY = drive1Y + drive2Y;

      // Normalize the combined drive vector
      if (driveX > 1) {
        driveY /= driveX;
        driveX = 1;
      } else if (driveX < -1) {
        driveY /= -driveX;
        driveX = -1;
      }
      if (driveY > 1) {
        driveX /= driveY;
        driveY = 1;
      } else if (driveY < -1) {
        driveX /= -driveY;
        driveY = -1;
      }

      drive.driveAutoRotate(driveX, driveY, rotationDeg, Auto.rotationToleranceDeg);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // do not do anything to drive for seamless use betweeen DriveManual
    if (interrupted) {
      DataLogManager.log("Interrupted directional rotation");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drive.isAtRotationTarget(Auto.rotationToleranceDeg)) {
      if (timeAtPos.hasElapsed(Auto.rotationTimeAtPosSec)) {
        return true;
      } else {
        timeAtPos.start();
        return false;
      }
    } else if (timeout.hasElapsed(Auto.rotationTimeoutSec)) {
      DataLogManager.log("Directional Rotation Timed Out");
      return true;
    } else {
      timeAtPos.stop();
      timeAtPos.reset();
      return false;
    }
  }
}
