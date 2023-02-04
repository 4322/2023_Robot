package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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
    if (Constants.joysticksEnabled) {
      
      // Joystick polarity:
      // Positive X is to the right
      // Positive Y is down
      // Positive Z is CW

      // WPI uses a trigonometric coordinate system with the front of the robot
      // pointing toward positive X. Thus:
      // Positive X is forward
      // Positive Y is to the left
      // Positive angles are CCW
      // Angles have a range of +/- 180 degrees (need to verify this)

      // All variables in this program use WPI coordinates

      // Cache hardware status for consistency in logic and convert
      // joystick coordinates to WPI coordinates.
      final double driveRawX = -RobotContainer.driveStick.getY();
      final double driveRawY = -RobotContainer.driveStick.getX();
      final double rotateRawX = -RobotContainer.rotateStick.getY();
      final double rotateRawY = -RobotContainer.rotateStick.getX();
      final double rotateRawZ = -RobotContainer.rotateStick.getZ();

      // calculate distance from center of joysticks
      final double driveRawR = Math.sqrt(driveRawX * driveRawX + driveRawY * driveRawY);
      final double rotateRawR = Math.sqrt(rotateRawX * rotateRawX + rotateRawY * rotateRawY);

      /* 
        cube drive joystick inputs to increase sensitivity
        x = smaller value
        y = greater value
        x = (y^3 / y) * x 
      */
      double driveX;
      double driveY;
      if (Math.abs(driveRawX) >= Math.abs(driveRawY)) {
        driveX = driveRawX * driveRawX * driveRawX;
        driveY = driveRawX * driveRawX * driveRawY;
      } else {
        driveX = driveRawY * driveRawY * driveRawX;
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