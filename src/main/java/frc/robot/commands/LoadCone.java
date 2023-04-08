package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class LoadCone extends InstantCommand {

  public LoadCone() {
  }

  @Override
  public void initialize() {
    LED.getInstance().setGamePiece(LED.GamePiece.cone);
  }
}
