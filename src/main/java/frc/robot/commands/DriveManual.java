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
  private final Double targetHeadingDeg;
  private boolean done = false;

  public DriveManual(Drive drivesubsystem, Double targetHeadingDeg) {
    drive = drivesubsystem;
    this.targetHeadingDeg = targetHeadingDeg;

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
      // All "theta" variables are in radians

      // Dual driver inputs need to be processed in an additive manner
      // instead of being averaged to avoid discontinuities.

      // Cache hardware status for consistency in logic and convert
      // joystick/Xbox coordinates to WPI coordinates.
      final double drive1RawX = -RobotContainer.driveStick.getY();
      final double drive1RawY = -RobotContainer.driveStick.getX();
      final double rotate1Raw = -RobotContainer.rotateStick.getZ();

      final double drive2RawX = -RobotContainer.xbox.getLeftY();
      final double drive2RawY = -RobotContainer.xbox.getLeftX();
      final double rotate2Raw = -RobotContainer.xbox.getRightX();

      // Deadbands are dependent on the type of input device
      final double drive1Deadband = Manual.joystickDriveDeadband;
      final double rotate1LeftDeadband = Manual.joystickRotateLeftDeadband;
      final double rotate1RightDeadband = Manual.joystickRotateRightDeadband;
      final double drive2Deadband = Manual.xboxDriveDeadband;
      final double rotate2LeftDeadband = Manual.xboxRotateDeadband;
      final double rotate2RightDeadband = Manual.xboxRotateDeadband;

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

      // Normalize the rotation inputs over deadband.
      // Process each input separately to avoid a discontinuity  
      // when the second input suddenly exceeds deadband.
      double rotatePower1 = 0;
      if (rotate1Raw > rotate1LeftDeadband) {
        rotatePower1 = (rotate1Raw - rotate1LeftDeadband) / (1 - rotate1LeftDeadband);
      } else if (rotate1Raw < -rotate1RightDeadband) {
        rotatePower1 = (rotate1Raw + rotate1RightDeadband) / (1 - rotate1RightDeadband);
      }
      rotatePower1 = rotatePower1 * rotatePower1 * rotatePower1;  // Increase sensitivity efficiently

      double rotatePower2 = 0;
      if (rotate2Raw > rotate2LeftDeadband) {
        rotatePower2 = (rotate2Raw - rotate2LeftDeadband) / (1 - rotate2LeftDeadband);
      } else if (rotate2Raw < -rotate2RightDeadband) {
        rotatePower2 = (rotate2Raw + rotate2RightDeadband) / (1 - rotate2RightDeadband);
      }
      rotatePower2 = rotatePower2 * rotatePower2 * rotatePower2;  // Increase sensitivity efficiently

      // Cap the combined rotation power
      double rotatePower = rotatePower1 + rotatePower2;
      if (rotatePower > 1) {
        rotatePower = 1;
      } else if (rotatePower < -1) {
        rotatePower = -1;
      }

      if (targetHeadingDeg == null) {
        drive.drive(driveX, driveY, rotatePower);
      } else if (rotatePower == 0) {
        drive.driveAutoRotate(driveX, driveY, targetHeadingDeg,
            Constants.DriveConstants.Manual.rotateToleranceDegrees);
      } else {
        // break out of auto-rotate and return to default command
       drive.drive(driveX, driveY, rotatePower);
       done = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
