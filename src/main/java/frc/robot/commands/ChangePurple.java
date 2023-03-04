package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
public class ChangePurple extends InstantCommand{
  LED LED;
  public ChangePurple (LED ledSubsystem) {
    LED = ledSubsystem;
    addRequirements(LED);
  }
  @Override
  public void initialize()
  {
    LED.purple();
  }
}
