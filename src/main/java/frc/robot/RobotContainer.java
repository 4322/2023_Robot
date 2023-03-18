package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private JoystickButton driveButtonNine;
  private JoystickButton driveButtonEleven;
  private JoystickButton driveButtonTwelve;
  private JoystickButton rotateButtonFour;
  private JoystickButton rotateButtonFive;
  private JoystickButton rotateButtonSix;

  private JoystickButton rotateTrigger;

  private ShuffleboardTab tab;
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();
  private final LED LED = new LED();
  private final PathPlannerManager ppManager;

  // Arm commands
  private final ArmRotateToPosition armRotateToLoadPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.loadPosition);
  private final ArmRotateToPosition armRotateToLoadHighPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.loadHighPosition);
  private final ArmRotateToPosition armRotateToMidPosition =
      new ArmRotateToPosition(arm, Constants.ArmConstants.midScoringPosition);
  private final ArmSetCoastMode armSetCoastMode = new ArmSetCoastMode(arm);

  // Claw commands
  private final ClawIntake clawIntake = new ClawIntake(claw);
  private final ClawOuttake clawOuttake = new ClawOuttake(claw);

  // Drive Commands
  private final DriveManual driveManualDefault = new DriveManual(drive, null);
  private final DriveManual driveManualForward = new DriveManual(drive, 0.0);
  private final DriveManual driveManualLeft = new DriveManual(drive, 90.0);
  private final DriveManual driveManualRight = new DriveManual(drive, -90.0);
  private final DriveStop driveStop = new DriveStop(drive);

  //LED Commands
  private final ChangeYellow changeYellow = new ChangeYellow(LED);
  private final ChangePurple changePurple = new ChangePurple(LED);

  // Auto Commands
  private final AutoBalance autoBalanceForward = new AutoBalance(drive, true);
  private final AutoBalance autoBalanceBackward = new AutoBalance(drive, false);
  private final AutoDriveOverCharge autoDriveOverChargeForward = new AutoDriveOverCharge(drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Initialize subsystems that use CAN after all of them have been constructed because the
    // constructors lower CAN bus utilization to make configuration reliable.
    drive.init();
    arm.init();
    claw.init();

    // Conifigure the button bindings
    configureButtonBindings();

    tab = Shuffleboard.getTab("Auto");
    
    tab.add("Auto Mode", autoChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 0)
      .withSize(4, 2);

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManualDefault);
    }

    if (Constants.armEnabled) {
      arm.setDefaultCommand(armRotateToLoadPosition);
    }

    ppManager = new PathPlannerManager(drive);

    ppManager.addEvent("initialize", new SequentialCommandGroup(
        new ArmHoming(arm)
      )
    );
    ppManager.addEvent("scoreCone", new SequentialCommandGroup(
        new ParallelRaceGroup(
          new AutoArmRotateToPosition(arm, Constants.ArmConstants.midScoringPosition), 
          new ClawIntake(claw)
        ),
        new TimedClawOuttake(claw, 0.5),
        new AutoArmRotateToPosition(arm, Constants.ArmConstants.loadPosition)
      )
    );
    
    autoChooser.addOption("DockCharge", 
      new SequentialCommandGroup(
        ppManager.loadAuto("DockCharge", false),
        new AutoDriveRotateWheels(drive, 0.25)
      )
    );

    autoChooser.addOption("ScorePreloadMobility", 
      ppManager.loadAuto("ScorePreload", false));
      
    autoChooser.addOption("ScorePreloadOnly", 
      new SequentialCommandGroup(
        new ArmHoming(arm),
        new ParallelRaceGroup(
          new AutoArmRotateToPosition(arm, Constants.ArmConstants.midScoringPosition), 
          new ClawIntake(claw)
        ),
        new TimedClawOuttake(claw, 0.5),
        new AutoArmRotateToPosition(arm, Constants.ArmConstants.loadPosition)
      )
    );
    
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
      driveButtonThree = new JoystickButton(driveStick, 3); //cone
      driveButtonFour = new JoystickButton(driveStick, 4); //cube
      driveButtonFive = new JoystickButton(driveStick, 5);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonNine = new JoystickButton(driveStick, 9);
      driveButtonEleven = new JoystickButton(driveStick, 11);
      driveButtonTwelve = new JoystickButton(driveStick, 12);
      rotateTrigger = new JoystickButton(rotateStick, 1);
      rotateButtonFour = new JoystickButton(rotateStick, 4);
      rotateButtonFive = new JoystickButton(rotateStick, 5);
      rotateButtonSix = new JoystickButton(rotateStick, 6);

      driveTrigger.whileTrue(clawOuttake);
      driveButtonThree.onTrue(changeYellow);
      driveButtonFour.onTrue(changePurple);
      driveButtonFive.onTrue(clawIntake);
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      driveButtonNine.onTrue(autoBalanceForward);
      driveButtonEleven.onTrue(autoBalanceBackward);
      driveButtonTwelve.onTrue(autoDriveOverChargeForward);
      rotateTrigger.whileTrue(armRotateToMidPosition);
      rotateButtonFour.onTrue(() -> armSetMidTarget());
      rotateButtonFive.onTrue(driveManualForward);
      rotateButtonSix.onTrue(armSetHighTarget);
    }

    if (Constants.xboxEnabled) {
      xbox.leftTrigger().onTrue(clawIntake);
      xbox.rightTrigger().whileTrue(clawOuttake);
      xbox.back().onTrue(armSetCoastMode);
      xbox.leftBumper().onTrue(driveManualLeft);
      xbox.rightBumper().onTrue(driveManualRight);
      xbox.a().whileTrue(armRotateToLoadHighPosition);
    }
  }

  private void armSetMidTarget() {

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
      driveStop.schedule();  // interrupt all drive commands
    }
    disableTimer.reset();
    disableTimer.start();
  }

  public Command getAutonomousCommand() {
    if (Constants.demo.inDemoMode) {
      return null;
    }

    return new SequentialCommandGroup(
      new ResetFieldCentric(drive, 0, true),
      autoChooser.getSelected()
    );
  }

  public void armReset() {
    new ArmHoming(arm).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule();
  }
}
