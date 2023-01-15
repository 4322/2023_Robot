package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
public class ClawIntakeIn extends CommandBase {
  private Claw claw;
  public ClawIntakeIn(Claw clawSubsystem)
  {
    claw = clawSubsystem;
    //addRequirements(claw);
  }
}
