package frc.robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  // Define controllers

  public static XboxController coPilot;
  public static Joystick driveStick;
  public static Joystick rotateStick;

  // Define buttons
  public static JoystickButton aButton;
  public static JoystickButton bButton;
  public static JoystickButton xButton;
  public static JoystickButton yButton;
  public static JoystickButton leftBumperButton;
  public static JoystickButton rightBumperButton;
  public static JoystickButton backButton;
  public static JoystickButton startButton;
  public static JoystickButton leftStickButton;
  public static JoystickButton rightStickButton;

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Drive drive = new Drive();
  private final DriveManual driveManual = new DriveManual(drive);

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
    
    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }
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

        JoystickButton aButton = new JoystickButton(coPilot, XboxController.Button.kA.value);
        JoystickButton bButton = new JoystickButton(coPilot, XboxController.Button.kB.value);
        JoystickButton xButton = new JoystickButton(coPilot, XboxController.Button.kX.value);
        JoystickButton yButton = new JoystickButton(coPilot, XboxController.Button.kY.value);
        JoystickButton leftBumperButton = new JoystickButton(coPilot, XboxController.Button.kLeftBumper.value);
        JoystickButton rightBumperButton = new JoystickButton(coPilot, XboxController.Button.kRightBumper.value);
        JoystickButton backButton = new JoystickButton(coPilot, XboxController.Button.kBack.value);
        JoystickButton startButton = new JoystickButton(coPilot, XboxController.Button.kStart.value);
        JoystickButton leftStickButton = new JoystickButton(coPilot, XboxController.Button.kLeftStick.value);
        JoystickButton rightStickButton = new JoystickButton(coPilot, XboxController.Button.kRightStick.value);

        JoystickButton driveButtonSeven = new JoystickButton(driveStick, 7);

        driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));

        if (coPilot.getAButtonPressed()){
          if (!coPilot.getAButtonReleased()){
            new ClawIntake(claw);
          }
        }

        if (coPilot.getBButtonPressed()){
          if (!coPilot.getBButtonReleased()){
            new ClawOuttake(claw);
          }
        } 
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

  public void enableSubsystems() {
    drive.setDriveMode(Drive.getDriveMode());  // reset limelight LED state
    drive.setBrakeMode();
    arm.setBrakeMode();
    claw.setBrakeMode();
  }

  public void disableSubsystems() {
    arm.setCoastMode();
    claw.setCoastMode();
    drive.stop();
  }

  // Stagger status frames from Talon FX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerTalonStatusFrames(WPI_TalonFX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }

  // Stagger status frames from Talon SRX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerTalonStatusFrames(WPI_TalonSRX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,
        RobotContainer.nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }
}
