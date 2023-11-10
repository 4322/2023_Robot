package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.PathConstraints;

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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.commands.DriveManual.AutoPose;
import frc.utility.Auto;
import frc.utility.OrangeSendableChooser;

public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
    //operator = 1
    //driver  = 2
  public static CommandXboxController xbox1 = new CommandXboxController(2);
  public static CommandXboxController xbox2 = new CommandXboxController(3);

  private ShuffleboardTab tab;
  private ArrayList<Auto> autoArrayList = new ArrayList<Auto>();
  private SendableChooser<Integer> positionChooser = new SendableChooser<Integer>();
  private OrangeSendableChooser<Command> autoChooser = new OrangeSendableChooser<Command>();

  private final Arm arm = Arm.getInstance();
  private final Telescope telescope = Telescope.getInstance();
  private final Claw claw = Claw.getInstance();
  private final Drive drive = new Drive();

  private final PathPlannerManager ppManager;

  // Arm commands
  private final ArmSetCoastMode armSetCoastMode = new ArmSetCoastMode(arm, telescope);
  private final ArmSetBrakeMode armSetBrakeMode = new ArmSetBrakeMode(arm, telescope);

  // Claw commands
  private final ClawOuttake clawOuttake = new ClawOuttake(false);
  private final ClawOuttake clawOuttakeForce = new ClawOuttake(true);

  // Drive Commands
  private final DriveManual driveManualDefault = new DriveManual(drive, DriveManual.AutoPose.none);
  private final DriveStop driveStop = new DriveStop(drive);

  // Auto Balance Commands

  private final SequentialCommandGroup autoBalanceForward = new SequentialCommandGroup(
      new AutoBalance(drive, true, false),
      new AutoDriveRotateWheels(drive, 0.25));
  private final SequentialCommandGroup autoBalanceBackward = new SequentialCommandGroup(
      new AutoBalance(drive, false, false),
      new AutoDriveRotateWheels(drive, 0.25));

  private int selectedPosition = 0;
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
    
    tab.add("Position", positionChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(0, 0)
      .withSize(3, 2);

    tab.add("Auto", autoChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(3, 0)
      .withSize(6, 2);

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManualDefault);
    }

    if (Constants.armEnabled) {
      arm.setDefaultCommand(new ArmMove(arm, telescope, ArmMove.Position.inBot, false));
    }

    if (Constants.gridLimeLightEnabled) {
      Limelight.getGridInstance().setDefaultCommand(new AlignAssistGrid());
    }

    if (Constants.substationLimeLightEnabled) {
      Limelight.getSubstationInstance().setDefaultCommand(new AlignAssistSubstation(drive));
    }

    ppManager = new PathPlannerManager(drive);

    ppManager.addEvent("scoreHigh", getScoreHigh());
    ppManager.addEvent("loadFloor", getLoadFloor());

    loadAutos();

    positionChooser.addOption("1", 1);
    positionChooser.addOption("2", 2);
    positionChooser.addOption("3", 3);
    positionChooser.addOption("4", 4);
    positionChooser.addOption("5", 5);
    positionChooser.addOption("6", 6);
    positionChooser.addOption("7", 7);
    positionChooser.addOption("8", 8);
    positionChooser.addOption("9", 9);
  }

  // autos need to be reloaded after each auto test because the commands can't be reused
  private void loadAutos() {
    
    // Rest autoArrayList and selectedPosition
    autoArrayList.clear();
    selectedPosition = 0;

    autoArrayList.add(new Auto("Do Nothing", new Nothing(), Arrays.asList(1, 2, 3, 4, 5, 6, 7, 8, 9)));

    autoArrayList.add(new Auto("Preload Only", getScoreHigh(), Arrays.asList(1, 2, 3, 4, 5, 6, 7, 8, 9)));

    autoArrayList.add(new Auto(
      "Mobility",
      new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityPreRotate1", false),
            new DriveManual(drive, AutoPose.loadSingleManual)
      ),
      Arrays.asList(1)
    ));

    autoArrayList.add(new Auto(
      "Mobility",
      new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityPreRotate9", false),
            new DriveManual(drive, AutoPose.loadSingleManual)
      ),
      Arrays.asList(9)
    ));

    autoArrayList.add(new Auto(
      "MobilityNotCracked",
      new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityOnlyNotCracked", false),
            new DriveManual(drive, AutoPose.loadSingleManual)
      ),
      Arrays.asList(1,9)
    ));

    autoArrayList.add(new Auto(
      "Mobility",
      ppManager.loadAuto("ScoreMobilityOnly2", false),
      Arrays.asList(2)
    ));

    autoArrayList.add(new Auto(
      "Mobility",
      ppManager.loadAuto("ScoreMobilityOnly8", false),
      Arrays.asList(8)
    ));

    autoArrayList.add(new Auto(
      "Score + Pickup",
      ppManager.loadAuto("ScorePickup1", false),
      Arrays.asList(1)
    ));

    autoArrayList.add(new Auto(
      "Score + Pickup",
      ppManager.loadAuto("ScorePickup9", false),
      Arrays.asList(9)
    ));

    autoArrayList.add(new Auto(
      "Mobility + Engage",
      new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityCharge1", false),
            new AutoBalance(drive, false, true),
            new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(1)
    ));

    autoArrayList.add(new Auto(
      "Mobility + Engage",
      new SequentialCommandGroup(
            ppManager.loadAuto("ScoreMobilityCharge2", false),
            new AutoBalance(drive, false, true),
            new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(2)
    ));

    autoArrayList.add(new Auto(
      "Mobility + Engage",
      new SequentialCommandGroup(
          ppManager.loadAuto("DriveOverCharge", false, 
            new PathConstraints(1.33, DriveConstants.Auto.autoMaxAccelerationMetersPerSec2)),
          new AutoBalance(drive, false, true),
          new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(4, 5, 6)
    ));

    autoArrayList.add(new Auto(
      "Mobility + Engage",
      new SequentialCommandGroup(
          ppManager.loadAuto("ScoreMobilityCharge8", false),
          new AutoBalance(drive, false, true),
          new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(8)
    ));

    autoArrayList.add(new Auto(
      "Mobility + Engage",
      new SequentialCommandGroup(
          ppManager.loadAuto("ScoreMobilityCharge9", false),
          new AutoBalance(drive, false, true),
          new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(9)
    ));

    autoArrayList.add(new Auto(
      "Engage", 
      new SequentialCommandGroup(
        getScoreHigh(),
        new AutoBalance(drive, true, true),
        new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(4, 5, 6)
    ));

    autoArrayList.add(new Auto(
      "2 Piece",
      ppManager.loadAuto("Cracked2PieceAuto1", false),
      Arrays.asList(1)
    ));

    autoArrayList.add(new Auto(
      "2 Piece",
      ppManager.loadAuto("Cracked2PieceAuto9", false),
      Arrays.asList(9)
    ));

    autoArrayList.add(new Auto(
      "2 Piece + Pickup",
      ppManager.loadAuto("2Pieces1Pickup1", false),
      Arrays.asList(1)
    ));

    autoArrayList.add(new Auto(
      "2 Piece + Pickup",
      ppManager.loadAuto("2Pieces1Pickup9", false),
      Arrays.asList(9)
    ));

    autoArrayList.add(new Auto(
      "2 Piece + Engage",
      new SequentialCommandGroup(
        ppManager.loadAuto("2PiecesEngage1", false),
        new AutoBalance(drive, true, true),
        new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(1)
    ));

    autoArrayList.add(new Auto(
      "2 Piece + Engage",
      new SequentialCommandGroup(
        ppManager.loadAuto("2PiecesEngage9", false),
        new AutoBalance(drive, true, true),
        new AutoDriveRotateWheels(drive, 0.25)
      ),
      Arrays.asList(9)
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {  
    BooleanSupplier isNotReAlignPreset = () -> ArmMove.isNotReAlignPreset();

    if (Constants.xboxEnabled) {
      //operator controls
      xbox1.back().onTrue(armSetCoastMode);
      xbox1.start().onTrue(armSetBrakeMode);
      xbox1.leftBumper().onTrue(Commands.runOnce(() -> LED.getInstance().setGamePiece(LED.GamePiece.cube)));
      xbox1.rightBumper().onTrue(Commands.runOnce(() -> LED.getInstance().setGamePiece(LED.GamePiece.cone)));
      xbox1.y().onTrue(new SetArmPreset(drive, ArmMove.Position.scoreHigh));
      xbox1.b().onTrue(new SetArmPreset(drive, ArmMove.Position.scoreMid));
      xbox1.a().onTrue(new SetArmPreset(drive, ArmMove.Position.scoreLow));
      xbox1.povDown().onTrue(new SetArmPreset(drive, ArmMove.Position.loadFloor));
      xbox1.povUp().onTrue(new SetArmPreset(drive, ArmMove.Position.loadSingleExtend));
      xbox1.povLeft().whileTrue(clawOuttakeForce);
      xbox1.povRight().onTrue(new TelescopeHoming(telescope, true)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

      //driver controls
      xbox2.leftTrigger().whileTrue(new DriveManual(drive, DriveManual.AutoPose.usePresetNoArmMove)
      .unless(isNotReAlignPreset));
      xbox2.leftTrigger().whileTrue(new ArmMove(arm, telescope, ArmMove.Position.usePreset, false));
      xbox2.rightTrigger().whileTrue(clawOuttake);
      xbox2.rightBumper().onTrue(new DriveManual(drive, DriveManual.AutoPose.usePresetManual));
      xbox2.povUp().onTrue(new ResetFieldCentric(drive, 0, true));
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

    updateChoosers();
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

    if (autoChooser.getSelected() == null) {
      return new Nothing();
    }

    return new SequentialCommandGroup(
      getAutoInitialize(),
      autoChooser.getSelected()
    );
  }

  public void homeArm() {
    new SequentialCommandGroup(
      new TelescopeHoming(telescope, false),
      new ArmHoming(arm)
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule();
  }

  // AUTO COMMANDS

  public void updateChoosers() {
    if (positionChooser.getSelected() != null) {
      if (positionChooser.getSelected() != selectedPosition) {
        selectedPosition = positionChooser.getSelected();
        autoChooser.removeAllOptions();
        for (Auto auto : autoArrayList) {
          if (auto.positions.contains(selectedPosition)) {
            autoChooser.addOption(auto.name, auto.command);
          }
        }
      }
    }
  }

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    return new SequentialCommandGroup(
      new ResetFieldCentric(drive, 0, true),
      new TelescopeHoming(telescope, false),
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
      new TimedClawOuttake(claw, DriveConstants.clawTimedOuttake),
      new ArmMove(arm, telescope, ArmMove.Position.inBot, true)
      );
  }

  //pick up gamw piece from floor
  public Command getLoadFloor() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new ArmMove(arm, telescope, ArmMove.Position.loadFloor, true), 
        new ClawIntake(claw)
      ),
      new ArmMove(arm, telescope, ArmMove.Position.inBot, true)
      );
  }
}
