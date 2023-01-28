package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawIntakeIn extends CommandBase {
  private Claw claw;

  public ClawIntakeIn(Claw clawSubsystem) {
    claw = clawSubsystem;

    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.manualIntake();
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.manualStop();
  }

  // Run until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
