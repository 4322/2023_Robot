package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

/* 
 * THIS COMMAND IS CURRENTLY UNUSABLE, DO NOT USE IT UNDER ANY CIRCUMSTANCES
*/


public class ArmManual extends CommandBase {

  private final Arm arm;
  private double speed;

  public ArmManual(Arm armsubsystem) {
    arm = armsubsystem;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = RobotContainer.xbox.getLeftY();
    if (Math.abs(speed) < ArmConstants.manualDeadband) {
      speed = 0;
    }
    // spread max climber power across full joystick range for increased sensitivity
    speed *= ArmConstants.kMaxRange;

    if (arm.getPosition() > ArmConstants.maxPosition) {
      DataLogManager.log("Going past max position");
      arm.stop();
    } else if (arm.getPosition() < ArmConstants.minPosition) {
      DataLogManager.log("Going past min position");
      arm.stop();
    } else {
      arm.setArmSpeed(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
