package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawOuttake extends CommandBase {
  private Claw claw;
  private boolean outtakeStarted;

  public ClawOuttake(Claw clawSubsystem) {
    claw = clawSubsystem;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    outtakeStarted = false;
  }

  @Override
  public void execute() {
    if (!outtakeStarted && ArmMove.isSafeToOuttake()) {
      claw.changeState(Claw.ClawMode.outtaking);
      outtakeStarted = true;
      ArmMove.setArmPreset(ArmMove.Position.loadSingleRetract);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (outtakeStarted) {
      claw.changeState(Claw.ClawMode.stopped);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
