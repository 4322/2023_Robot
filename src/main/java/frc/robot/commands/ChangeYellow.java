package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
public class ChangeYellow extends InstantCommand{
  LED LED;
  public ChangeYellow (LED ledSubsystem) {
    LED = ledSubsystem;
    addRequirements(LED);
  }
  @Override
  public void initialize()
  {
    LED.yellow();
  }
}
