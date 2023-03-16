package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  private boolean done;
  Timer secondDeadBandTimer = new Timer();
  private spinoutMode currentMode;

  public enum spinoutMode {
    none,
    frontLeftCW,
    frontLeftCCW,
    backLeftCW,
    backLeftCCW,
    backRightCW,
    backRightCCW,
    frontRightCW,
    frontRightCCW,
    done,
    abort;
  }

  public DriveManual(Drive drivesubsystem, Double targetHeadingDeg) {
    drive = drivesubsystem;
    this.targetHeadingDeg = targetHeadingDeg;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetRotatePID();
    done = false;  // make command reusable
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
      
      if (rotate1Raw > Constants.DriveConstants.Manual.spinoutRotateToleranceDegrees) {
        secondDeadBandTimer.start();
      } else {
        secondDeadBandTimer.restart();
      }

      if (rotate2Raw > Constants.DriveConstants.Manual.spinoutRotateToleranceDegrees) {
        secondDeadBandTimer.start();
      } else {
        secondDeadBandTimer.restart();
      }

      
      // detect if not rotating and if rotate stick past second deadband for certain amount of time
      //    (first deadband is rotateToleranceDegrees/xboxRotateDeadband)
      //    (second deadband is past first deadband in rotation) (close to max rotation)
      if (drive.getAngleVelocity() < Constants.DriveConstants.Manual.spinoutMinAngleVelocity &&
          secondDeadBandTimer.hasElapsed(Constants.DriveConstants.Manual.spinoutSecondDeadBandThreshold)) {
            // from this, figure out which swerve module to lock onto to rotate off of (use drive stick direction and robotAngle)
            //    How to use drive stick: module closest to direction of drivestick. 
            //      use gyro to find orientation
            //      algorithm to determine quadrant: driveStickAngle - robotAngle (TBD)
            //        if drivestick angle 0 < x < 90 , in quadrant 1 (front left module)
            //        if drivestick angle 90 < x < 180 , in quadrant 2 (back left module)
            //        if drivestick angle -180 < x < -90 , in quadrant 3 (back right module)
            //        if drivestick angle -90 < x < 0 , in quadrant 4 (front right module)


            // use state machine for rotating each wheel in each direction (8 cases)
            //    each module rotating CW and CCW
            //      if rotation stick falls under second deadband or robot rotates 90 degrees, 
            //      reset rotation back to normal
            switch (currentMode) {
              case none:
              case frontLeftCW:
              case frontLeftCCW:
              case backLeftCW:
              case backLeftCCW:
              case backRightCW:
              case backRightCCW:
              case frontRightCW:
              case frontRightCCW:
              case done:
              case abort:
            } // pain :(
            // SPECIAL CASE: if driveStickAngle - robotAngle is exactly 0, 90, 180, -180, then use the rotate angle to determine wheel:
            //                  0: if CW, quadrant 1 (front left); if CCW, quadrant 4 (front right)
            //                  90: if CW, quadrant 2 (back left); if CCW, quadrant 1 (front left)
            //                  180/-180: if CW, quadrant 3 (back right); if CCW, quadrant 2 (back left)
            //                  -90: if CW, quadrant 4 (front right); if CCW, quadrant 3 (back right)
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
