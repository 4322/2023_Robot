package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LED;

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
      LED.getInstance().setLastGamePiece();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // don't stop the claw because power is required to retain the game piece
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
