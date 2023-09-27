package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawIntake extends CommandBase {
  private Claw claw;
  private boolean intakeStalled;

  public ClawIntake(Claw clawSubsystem) {
    claw = clawSubsystem;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.changeState(Claw.ClawMode.intaking);
    intakeStalled = false;
  }

  @Override
  public void execute() {
    if (!intakeStalled && Claw.getInstance().isIntakeStalled()) {
      ArmMove.setArmPresetToLastScorePreset();
      intakeStalled = true;
    }
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
