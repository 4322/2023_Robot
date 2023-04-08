package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class LoadCube extends InstantCommand {

  public LoadCube() {
  }

  @Override
  public void initialize() {
    LED.getInstance().setGamePiece(LED.GamePiece.cube);
  }
}
