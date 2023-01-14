package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  
  private Timer disableTimer = new Timer();

  // Define controllers
  public static XboxController coPilot = new XboxController(2);

  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();

  // Arm commands
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

  }

  // Initialize subsystems that use CAN after all of them have been constructed because the 
  // constructors lower CAN bus utilization to make configuration reliable.
    
  // Configure the button bindings
  
  
  public void disableSubsystems() {
    arm.setCoastMode();
  }
}
