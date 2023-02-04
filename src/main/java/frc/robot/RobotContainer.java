package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  private Timer disableTimer = new Timer();

  // Define controllers
  public static Joystick driveStick;
  public static Joystick rotateStick;

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  // Arm commands
  // Claw commands
  private final ClawIntake clawIntake = new ClawIntake(claw);
  private final ClawOuttake clawOuttake = new ClawOuttake(claw);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

  }

  // Initialize subsystems that use CAN after all of them have been constructed because the
  // constructors lower CAN bus utilization to make configuration reliable.

  // Configure the button bindings
  
  // stagger status frame periods to reduce peak CAN bus utilization
  private static int nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;

  public static int nextFastStatusPeriodMs() {
    if (nextFastStatusPeriodMs > Constants.fastStatusPeriodMaxMs) {
      nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
    }
    return nextFastStatusPeriodMs++;
  }
  public static int nextSlowStatusPeriodMs() {
    if (nextSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSlowStatusPeriodMs++;
  }
  public void disableSubsystems() {
    arm.setCoastMode();
    claw.setCoastMode();
  }
}
