package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class ChangeYellow extends InstantCommand {
  private LED LED;
  private Limelight limelight;
  private int pipeline;

  public ChangeYellow(LED ledSubsystem, Limelight gridLimelight, int limelightPipeline) {
    LED = ledSubsystem;
    limelight = gridLimelight;
    pipeline = limelightPipeline;
  }

  @Override
  public void initialize() {
    LED.yellow();
    limelight.switchPipeline(pipeline);
  }
}
