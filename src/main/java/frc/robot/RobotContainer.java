package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.commands.*;

public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
  public static CommandXboxController xbox = new CommandXboxController(2);
  public static Joystick driveStick;
  public static Joystick rotateStick;

  private JoystickButton driveTrigger;
  private JoystickButton driveButtonThree;
  private JoystickButton driveButtonFour;
  private JoystickButton driveButtonFive;
  private JoystickButton driveButtonSeven;
  private JoystickButton rotateButtonFive;

  private JoystickButton rotateTrigger;

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();
  private final LED LED = new LED();

  // Arm commands
  private final ArmRotateToPosition armRotateToLoadPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.LoadPosition);
  private final ArmRotateToPosition armRotateToLoadHighPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.LoadHighPosition);
  private final ArmRotateToPosition armRotateToMidPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.MidScoringPosition);
  private final ArmSetCoastMode armSetCoastMode = new ArmSetCoastMode(arm);

  // Claw commands
  private final ClawIntake clawIntake = new ClawIntake(claw);
  private final ClawOuttake clawOuttake = new ClawOuttake(claw);

  // Drive Commands
  private final DriveManual driveManualDefault = new DriveManual(drive, null);
  private final DriveManual driveManualForward = new DriveManual(drive, 0.0);
  private final DriveManual driveManualLeft = new DriveManual(drive, 90.0);
  private final DriveManual driveManualRight = new DriveManual(drive, -90.0);

  //LED Commands
  private final ChangeYellow changeYellow = new ChangeYellow(LED);
  private final ChangePurple changePurple = new ChangePurple(LED);
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
      drive.setDefaultCommand(driveManualDefault);
    }

    if (Constants.armEnabled) {
      arm.setDefaultCommand(armRotateToLoadPosition);
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

      driveTrigger = new JoystickButton(driveStick, 1);
      driveButtonThree = new JoystickButton(driveStick, 3);//cone
      driveButtonFour = new JoystickButton(driveStick, 4);//cube
      driveButtonFive = new JoystickButton(driveStick, 5);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      rotateTrigger = new JoystickButton(rotateStick, 1);
      rotateButtonFive = new JoystickButton(rotateStick, 5);

      driveTrigger.whileTrue(clawOuttake);
      driveButtonThree.onTrue(changeYellow);
      driveButtonFour.onTrue(changePurple);
      driveButtonFive.onTrue(clawIntake);
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      rotateTrigger.whileTrue(armRotateToMidPosition);
      rotateButtonFive.onTrue(driveManualForward);
    }

    if (Constants.xboxEnabled) {
      xbox.leftTrigger().onTrue(clawIntake);
      xbox.rightTrigger().whileTrue(clawOuttake);
      xbox.back().onTrue(armSetCoastMode);
      xbox.leftBumper().onTrue(changeYellow);
      xbox.rightBumper().onTrue(changePurple);
      xbox.a().whileTrue(armRotateToLoadHighPosition);
      xbox.x().onTrue(driveManualLeft);
      xbox.b().onTrue(driveManualRight);
    }
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
      claw.changeState(Claw.ClawMode.stopped);
      claw.setCoastMode();
    }
    if (Constants.driveEnabled) {
      drive.stop();
    }
    disableTimer.reset();
    disableTimer.start();
  }

  public void armReset() {
    new ArmHoming(arm).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
  }
}
