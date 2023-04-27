package frc.robot;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;
import frc.robot.Constants.DriveConstants;
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
  private JoystickButton driveButtonSix;
  private JoystickButton driveButtonSeven;
  private JoystickButton driveButtonNine;
  private JoystickButton driveButtonEleven;
  private JoystickButton driveButtonTwelve;

  private JoystickButton rotateTrigger;
  private JoystickButton rotateButtonThree;
  private JoystickButton rotateButtonFour;
  private JoystickButton rotateButtonFive;
  private JoystickButton rotateButtonSix;

  private ShuffleboardTab tab;
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Telescope telescope = new Telescope();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();

  private final PathPlannerManager ppManager;

  // Arm commands
  private final ArmSetCoastMode armSetCoastMode = new ArmSetCoastMode(arm, telescope);
  private final ArmSetBrakeMode armSetBrakeMode = new ArmSetBrakeMode(arm, telescope);

  // Claw commands
  private final ClawIntake clawIntake = new ClawIntake(claw);
  private final ClawOuttake clawOuttake = new ClawOuttake(claw);

  // Drive Commands
  private final DriveManual driveManualDefault = new DriveManual(drive, DriveManual.AutoPose.none);
  private final DriveManual driveManualForward = new DriveManual(drive, DriveManual.AutoPose.forward);
  private final DriveManual driveManualBackward = new DriveManual(drive, DriveManual.AutoPose.back);
  private final DriveManual driveManualLeft = new DriveManual(drive, DriveManual.AutoPose.left);
  private final DriveManual driveManualRight = new DriveManual(drive, DriveManual.AutoPose.right);
  private final DriveStop driveStop = new DriveStop(drive);

  // Auto Balance Commands
  private final SequentialCommandGroup autoBalanceForward = new SequentialCommandGroup(
      new AutoBalance(drive, true),
      new AutoDriveRotateWheels(drive, 0.25));
  private final SequentialCommandGroup autoBalanceBackward = new SequentialCommandGroup(
      new AutoBalance(drive, false),
      new AutoDriveRotateWheels(drive, 0.25));

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
      .withSize(9, 2);

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManualDefault);
    }

    if (Constants.armEnabled) {
      arm.setDefaultCommand(new ArmMove(arm, telescope, ArmMove.Position.inHopper, false));
    }

    if (Constants.gridLimeLightEnabled) {
      Limelight.getGridInstance().setDefaultCommand(new AlignAssistGrid());
    }

    if (Constants.substationLimeLightEnabled) {
      Limelight.getSubstationInstance().setDefaultCommand(new AlignAssistSubstation());
    }

    ppManager = new PathPlannerManager(drive);

    loadAutos();
  }

  // autos need to be reloaded after each auto test because the commands can't be reused
  private void loadAutos() {
    
    ppManager.addEvent("scoreCone", getScoreHigh());

    autoChooser.setDefaultOption("Do Nothing", new Nothing());

    autoChooser.addOption("Preload Only", getScoreHigh());

    autoChooser.addOption("Mobility (1, 9)",
        ppManager.loadAuto("ScoreMobilityOnly", false));

    autoChooser.addOption("Engage (1)",
        new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityCharge1", false),
            new AutoBalance(drive, false),
            new AutoDriveRotateWheels(drive, 0.25)
        )
      );

    autoChooser.addOption("Mobility (2)",
        ppManager.loadAuto("ScoreMobilityOnly2", false));

    autoChooser.addOption("Engage (2)",
        new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityCharge2", false),
            new AutoBalance(drive, false),
            new AutoDriveRotateWheels(drive, 0.25)
        )
      ); 

    autoChooser.addOption("Engage (4, 5, 6)",
      new SequentialCommandGroup(
          ppManager.loadAuto("DriveOverCharge", false, 
            new PathConstraints(1.33, DriveConstants.Auto.autoMaxAccelerationMetersPerSec2)),
          new AutoBalance(drive, false),
          new AutoDriveRotateWheels(drive, 0.25)
      )
    );

    autoChooser.addOption("Engage (8)",
      new SequentialCommandGroup(
          ppManager.loadAuto("ScoreMobilityCharge8", false),
          new AutoBalance(drive, false),
          new AutoDriveRotateWheels(drive, 0.25)
      )
    );

    autoChooser.addOption("Engage (9)",
      new SequentialCommandGroup(
          ppManager.loadAuto("ScoreMobilityCharge9", false),
          new AutoBalance(drive, false),
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
    BooleanSupplier isIntakeStalled = () -> claw.isIntakeStalled();
    BooleanSupplier isBackwardScoringPreset = () -> ArmMove.isBackwardScoringPreset();

    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);

      driveTrigger = new JoystickButton(driveStick, 1);
      driveButtonThree = new JoystickButton(driveStick, 3); //cone
      driveButtonFour = new JoystickButton(driveStick, 4); //cube
      driveButtonFive = new JoystickButton(driveStick, 5);
      driveButtonSix = new JoystickButton(driveStick, 6);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonNine = new JoystickButton(driveStick, 9);
      driveButtonEleven = new JoystickButton(driveStick, 11);
      driveButtonTwelve = new JoystickButton(driveStick, 12);
      rotateTrigger = new JoystickButton(rotateStick, 1);
      rotateButtonThree = new JoystickButton(rotateStick, 3);
      rotateButtonFour = new JoystickButton(rotateStick, 4);
      rotateButtonFive = new JoystickButton(rotateStick, 5);
      rotateButtonSix = new JoystickButton(rotateStick, 6);

      driveTrigger.whileTrue(clawOuttake);
      driveButtonThree.onTrue(Commands.runOnce(() -> LED.getInstance().setGamePiece(LED.GamePiece.cone)));
      driveButtonFour.onTrue(Commands.runOnce(() -> LED.getInstance().setGamePiece(LED.GamePiece.cube)));
      driveButtonFive.onTrue(clawIntake);
        driveButtonFive.onTrue(new ArmMove(arm, telescope, ArmMove.Position.loadSingle, false)
            .unless(isIntakeStalled).until(isIntakeStalled));
      driveButtonSix.onTrue(clawIntake);
        driveButtonSix.onTrue(new ArmMove(arm, telescope, ArmMove.Position.loadDouble, false)
            .unless(isIntakeStalled).until(isIntakeStalled));
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      driveButtonNine.onTrue(autoBalanceForward);
      driveButtonEleven.onTrue(autoBalanceBackward);
      driveButtonTwelve.onTrue(driveStop);

      // Re-establish alignment to grid when deploying the arm
      rotateTrigger.whileTrue(new DriveManual(drive, DriveManual.AutoPose.forward)
          .unless(isBackwardScoringPreset));
      rotateTrigger.whileTrue(new ArmMove(arm, telescope, ArmMove.Position.scorePreset, false)
          .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

      rotateButtonThree.onTrue(driveManualForward);
      rotateButtonThree.onTrue(new SetScoringPosition(ArmMove.Position.scoreMid));

      rotateButtonFour.onTrue(driveManualForward);
      rotateButtonFour.onTrue(new SetScoringPosition(ArmMove.Position.scoreHigh));

      rotateButtonFive.onTrue(driveManualBackward);
      rotateButtonFive.onTrue(new SetScoringPosition(ArmMove.Position.scoreLow));
      rotateButtonSix.onTrue(clawIntake);
        rotateButtonSix.onTrue(new ArmMove(arm, telescope, ArmMove.Position.loadFloor, false)
            .unless(isIntakeStalled).until(isIntakeStalled));
    }

    if (Constants.xboxEnabled) {
      xbox.leftTrigger().onTrue(clawIntake);
      if (Constants.useLoadHighPosition) {
        xbox.leftTrigger().onTrue(new ArmMove(arm, telescope, ArmMove.Position.loadSingle, false)
            .unless(isIntakeStalled).until(isIntakeStalled));
      } else {
        xbox.a().whileTrue(new ArmMove(arm, telescope, ArmMove.Position.loadBounce, false));
      }
      xbox.rightTrigger().whileTrue(clawOuttake);
      xbox.back().onTrue(armSetCoastMode);
      xbox.start().onTrue(armSetBrakeMode);
      xbox.leftBumper().onTrue(driveManualLeft);
      xbox.rightBumper().onTrue(driveManualRight);
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
    loadAutos();
  }

  public Command getAutonomousCommand() {
    if (Constants.Demo.inDemoMode) {
      return null;
    }

    return new SequentialCommandGroup(
      getAutoInitialize(),
      autoChooser.getSelected()
    );
  }

  public void homeArm() {
    new SequentialCommandGroup(
      new TelescopeHoming(telescope),
      new ArmHoming(arm)
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule();
  }

  // AUTO COMMANDS

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    return new SequentialCommandGroup(
      new ResetFieldCentric(drive, 0, true),
      new TelescopeHoming(telescope),
      new ArmHoming(arm) 
    );
  }

  // Score a game piece high
  public Command getScoreHigh() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new ArmMove(arm, telescope, ArmMove.Position.scoreHigh, true), 
        new ClawIntake(claw)
      ),
      new TimedClawOuttake(claw, 0.5),
      new ArmMove(arm, telescope, ArmMove.Position.inHopper, true)
      );
  }
}
