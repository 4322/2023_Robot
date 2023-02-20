package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmRotateToPosition extends CommandBase{
  private final Arm arm;
  double targetPos;
  public ArmRotateToPosition (Arm armSubsystem, double targetPos) {
    arm = armSubsystem;
    this.targetPos = targetPos;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.rotateToPosition(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtTarget();
  }
}
