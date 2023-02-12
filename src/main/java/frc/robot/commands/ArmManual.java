package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

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
    speed = RobotContainer.coPilot.getLeftY();
    if (Math.abs(speed) < ArmConstants.manualDeadband) {
      speed = 0;
    }
    // spread max climber power across full joystick range for increased sensitivity
    speed *= ArmConstants.kMaxRange;

    arm.setArmSpeed(speed);
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
