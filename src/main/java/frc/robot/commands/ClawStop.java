package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;
  
public class ClawStop extends InstantCommand {

  public ClawStop() {
    addRequirements(Claw.getInstance());
  }

  @Override
  public void initialize() {
    Claw.getInstance().changeState(Claw.ClawMode.stopped);
    ArmMove.setArmPreset(ArmMove.Position.loadSingleExtend);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}