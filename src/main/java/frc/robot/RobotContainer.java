package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.commands.*;

public class RobotContainer {

  private Timer disableTimer = new Timer();

  // Define controllers

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  // Arm commands
  // Claw commands
  private final ClawIntake clawIntakeIn = new ClawIntake(claw);
  private final ClawOuttake clawIntakeOut = new ClawOuttake(claw);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

  }

  // Initialize subsystems that use CAN after all of them have been constructed because the
  // constructors lower CAN bus utilization to make configuration reliable.

  // Configure the button bindings


  public void disableSubsystems() {
    arm.setCoastMode();
    claw.setCoastMode();
  }
}
