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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Telescope;
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

  private JoystickButton rotateTrigger;
  private JoystickButton rotateButtonThree;
  private JoystickButton rotateButtonFour;
  private JoystickButton rotateButtonFive;

  private ShuffleboardTab tab;
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Telescope telescope = new Telescope();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();
  private final LED LED = new LED();
  private final PathPlannerManager ppManager;

  // Arm commands
  private final ArmSetCoastMode armSetCoastMode = new ArmSetCoastMode(arm, telescope);

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    // Initialize subsystems that use CAN after all of them have been constructed because the
    // constructors lower CAN bus utilization to make configuration reliable.
    drive.init();  // must be first to initialize the CANcoders
    arm.init();
    telescope.init();
    claw.init();

    // Conifigure the button bindings
    configureButtonBindings();

    tab = Shuffleboard.getTab("Auto");
    
    tab.add("Auto Mode", autoChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 0)
      .withSize(6, 2);

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManualDefault);
    }

    if (Constants.armEnabled) {
      arm.setDefaultCommand(new ArmMove(arm, telescope, Constants.ArmConstants.loadPosition,
          Constants.Telescope.loadPosition, false));
    }

    ppManager = new PathPlannerManager(drive);

    ppManager.addEvent("initialize", new SequentialCommandGroup(
        new TelescopeHoming(telescope),
        new ArmHoming(arm)
      )
    );
    
    ppManager.addEvent("scoreCone", new SequentialCommandGroup(
        new ParallelRaceGroup(
          new ArmMove(arm, telescope, Constants.ArmConstants.highScoringPosition, Constants.Telescope.highScoringPosition, true), 
          new ClawIntake(claw)
        ),
        new TimedClawOuttake(claw, 0.5),
        new ArmMove(arm, telescope, Constants.ArmConstants.loadPosition, Constants.Telescope.loadPosition, true)
      )
    );

    autoChooser.setDefaultOption("Nothing", new Nothing());

    autoChooser.addOption("Score Preload & Mobility",
        ppManager.loadAuto("ScoreMobilityOnly", false));

    autoChooser.addOption("Engage Blue 9",
        new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityCharge9", false),
            new AutoBalance(drive, false),
            new AutoDriveRotateWheels(drive, 0.25)
          ));

    autoChooser.addOption("Engage Red 9",
        new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityCharge9", true),
            new AutoBalance(drive, false),
            new AutoDriveRotateWheels(drive, 0.25)
          ));    
      
    autoChooser.addOption("Score Preload Only", 
      new SequentialCommandGroup(
        ppManager.getEvent("initialize"),
        ppManager.getEvent("scoreCone")
      )
    );
    
    autoChooser.addOption("Auto Balance Forward", 
      new SequentialCommandGroup(
        ppManager.getEvent("initialize"),
        ppManager.getEvent("scoreCone"),
        new AutoBalance(drive, true),
        new AutoDriveRotateWheels(drive, 0.25)
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
      rotateButtonThree = new JoystickButton(rotateStick, 3);
      rotateButtonFour = new JoystickButton(rotateStick, 4);
      rotateButtonFive = new JoystickButton(rotateStick, 5);

      driveTrigger.whileTrue(clawOuttake);
      driveButtonThree.onTrue(changeYellow);
      driveButtonFour.onTrue(changePurple);
      driveButtonFive.onTrue(clawIntake);
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      driveButtonNine.onTrue(autoBalanceForward);
      driveButtonEleven.onTrue(autoBalanceBackward);
      driveButtonTwelve.onTrue(driveStop);
      rotateTrigger.whileTrue(new ArmMove(arm, telescope));
      rotateButtonThree.onTrue(new SetScoringTargets(arm, telescope, Constants.ArmConstants.midScoringPosition,
        Constants.Telescope.midScoringPosition));
      rotateButtonFour.onTrue(new SetScoringTargets(arm, telescope, Constants.ArmConstants.highScoringPosition,
        Constants.Telescope.highScoringPosition));
      rotateButtonFive.onTrue(driveManualForward);
    }

    if (Constants.xboxEnabled) {
      xbox.leftTrigger().onTrue(clawIntake);
      xbox.rightTrigger().whileTrue(clawOuttake);
      xbox.back().onTrue(armSetCoastMode);
      xbox.leftBumper().onTrue(driveManualLeft);
      xbox.rightBumper().onTrue(driveManualRight);
      xbox.a().whileTrue(new ArmMove(arm, telescope, Constants.ArmConstants.loadHighPosition,
          Constants.Telescope.loadPosition, false));
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
    telescope.setBrakeMode();
    claw.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    arm.stop();
    telescope.stop();
    claw.changeState(Claw.ClawMode.stopped);
    claw.setCoastMode();
    driveStop.schedule();  // interrupt all drive commands
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
    new SequentialCommandGroup(
      new TelescopeHoming(telescope),
      new ArmHoming(arm)
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule();
  }
}
