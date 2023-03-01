package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.commands.*;

public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
  public static CommandXboxController coPilot = new CommandXboxController(2);
  public static Joystick driveStick;
  public static Joystick rotateStick;
  private JoystickButton driveButtonSeven;

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();
  private final LED LED = new LED();

  // Arm commands
  private final ArmManual armManual = new ArmManual(arm);
  private final ArmRotateToPosition armRotateToLoadPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.LoadPosition);
  private final ArmRotateToPosition armRotateToMidPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.MidScoringPosition);
  private final ArmRotateToPosition armRotateToHighPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.HighScoringPosition);
  private final ArmSetCoastMode armSetCoastMode = new ArmSetCoastMode(arm);
  private final ArmHoming armHoming = new ArmHoming(arm);

  // Claw commands
  private final ClawIntake clawIntake = new ClawIntake(claw);
  private final ClawOuttake clawOuttake = new ClawOuttake(claw);

  // Drive Commands
  private final DriveManual driveManual = new DriveManual(drive);
  //LED Commands
  private final ColorChange colorChange = new ColorChange(LED);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Initialize subsystems that use CAN after all of them have been constructed because the
    // constructors lower CAN bus utilization to make configuration reliable.
    drive.init();
    arm.init();
    claw.init();

    // Conifigure the button bindings
    configureButtonBindings();

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }

    if (Constants.armEnabled) {
      //arm.setDefaultCommand(armManual);
    }
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {

    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
    }
    coPilot.leftTrigger().whileTrue(clawIntake);
    coPilot.rightTrigger().whileTrue(clawOuttake);
    coPilot.back().onTrue(armSetCoastMode);
    coPilot.a().onTrue(armRotateToLoadPosition);
    coPilot.b().onTrue(armRotateToMidPosition);
    coPilot.y().onTrue(armRotateToHighPosition);
    coPilot.x().onTrue(colorChange);
  }

  public void disabledPeriodic() {
    if (disableTimer.hasElapsed(Constants.DriveConstants.disableBreakSec)) {
      if (Constants.driveEnabled) {
        drive.setCoastMode(); // robot has stopped, safe to enter coast mode
      }
      disableTimer.stop();
      disableTimer.reset();
    }
  }

  public void enableSubsystems() {
    drive.setDriveMode(Drive.getDriveMode()); // reset limelight LED state
    drive.setBrakeMode();
    arm.setBrakeMode();
    claw.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    if (Constants.armEnabled) {
      arm.stop();
    }
    if (Constants.clawEnabled) {
      claw.stop();
      claw.setCoastMode();
    }
    if (Constants.driveEnabled) {
      drive.stop();
    }
    disableTimer.reset();
    disableTimer.start();
  }
}
