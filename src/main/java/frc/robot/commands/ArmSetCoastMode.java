package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ArmSetCoastMode extends InstantCommand{
  private Arm arm;

  public ArmSetCoastMode (Arm armsubsystem) {
    arm = armsubsystem;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    if(DriverStation.isDisabled())
    {
      arm.setCoastMode();
    }
  }
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}