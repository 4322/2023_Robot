// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private Command m_autonomousCommand;
  private ShuffleboardTab tab;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    tab = Shuffleboard.getTab("Enabled Subsystems");

    subsystemEnabled("Comp Mode", 0, 0, 
      !Constants.debug && !Constants.inDemoMode && !Constants.armTuningMode && !Constants.clawTuningMode);
    subsystemEnabled("Drivebase", 1, 0, Constants.driveEnabled);
    subsystemEnabled("Arm", 2, 0, Constants.armEnabled);
    subsystemEnabled("Arm Sensor", 3, 0, Constants.armSensorEnabled);
    subsystemEnabled("Claw", 4, 0, Constants.clawEnabled);
    subsystemEnabled("LEDs", 5, 0, Constants.ledEnabled);

    subsystemEnabled("Joysticks", 0, 1, Constants.joysticksEnabled);
    subsystemEnabled("Gyro", 1, 1, Constants.gyroEnabled);
    subsystemEnabled("Limeight", 2, 1, Constants.limeLightEnabled);
    subsystemEnabled("XBOX Controller", 3, 1, Constants.xboxEnabled);
    subsystemEnabled("Color Sensor", 4, 1, Constants.colorSensorEnabled);

    m_robotContainer = new RobotContainer();
  }

    // create new shuffleboard tab to show whether or not subsystem is enabled
  // also print error to driver station if not
  private void subsystemEnabled(String title, int posX, int posY, boolean enabled) {
    tab.add(title, enabled)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(posX, posY)
    .withSize(1, 1);

    if (!enabled) {
      DriverStation.reportError(title + " not enabled", false);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.enableSubsystems();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.enableSubsystems();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableSubsystems();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
