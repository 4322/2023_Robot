package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawOuttake extends CommandBase {
  private Claw claw;

  public ClawOuttake(Claw clawSubsystem) {
    claw = clawSubsystem;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    if (ArmMove.isSafeToOuttake()) {
      claw.changeState(Claw.ClawMode.outtaking);
    }
  }

  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.changeState(Claw.ClawMode.stopped);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
