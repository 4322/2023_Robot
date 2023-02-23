package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;
public class ColorChange extends InstantCommand{
  LED LED;
  public ColorChange (LED ledSubsystem) {
    LED = ledSubsystem;
    addRequirements(LED);
  }
  @Override
  public void initialize()
  {
    if (LED.ledColor() == LEDColor.purple)
      LED.yellow();
    else if (LED.ledColor() == LEDColor.yellow || LED.ledColor() == LEDColor.none)
      LED.purple();
  }
}
