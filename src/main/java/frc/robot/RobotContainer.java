package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.commands.*;

public class RobotContainer {

  private Timer disableTimer = new Timer();

  // Define controllers
  public static XboxController coPilot;
  public static Joystick driveStick;
  public static Joystick rotateStick;
  
  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();

  // Arm commands

  // Claw commands
  private final ClawIntake clawIntake = new ClawIntake(claw);
  private final ClawOuttake clawOuttake = new ClawOuttake(claw);

  // Drive Commands

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Initialize subsystems that use CAN after all of them have been constructed because the
    // constructors lower CAN bus utilization to make configuration reliable.
    drive.init();
    arm.init();
    claw.init();

    // Conifigure the button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
    private void configureButtonBindings() {
      
      if (Constants.joysticksEnabled) {
        driveStick = new Joystick(0);
        rotateStick = new Joystick(1);
        coPilot = new XboxController(2);
      }
    }
  
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
