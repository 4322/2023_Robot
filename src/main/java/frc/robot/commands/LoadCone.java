package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class LoadCone extends InstantCommand {
  private Limelight limelight;
  private int pipeline;

  public LoadCone(Limelight gridLimelight, int limelightPipeline) {
    limelight = gridLimelight;
    pipeline = limelightPipeline;
  }

  @Override
  public void initialize() {
    LED.getInstance().setGamePiece(LED.GamePiece.cone);
    limelight.switchPipeline(pipeline);
  }
}
