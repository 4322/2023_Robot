package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
public class ClawIntakeIn extends CommandBase {
  private Claw claw;
  public ClawIntakeIn(Claw clawSubsystem)
  {
    claw = clawSubsystem;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.Intake(); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stop();
  }
  @Override 
  public boolean isFinished() {
   return true;
  }
}
