package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class ArmSetCoastMode extends InstantCommand{
  private Arm arm;
  private Telescope telescope;

  public ArmSetCoastMode (Arm arm, Telescope telescope) {
    this.arm = arm;
    this.telescope = telescope;
    addRequirements(arm, telescope);
  }

  @Override
  public void initialize() {
    if(DriverStation.isDisabled())
    {
      arm.setCoastMode();
      telescope.setCoastMode();
    }
  }
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}