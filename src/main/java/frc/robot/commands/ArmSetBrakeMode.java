package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;

public class ArmSetBrakeMode extends InstantCommand{
  private Arm arm;
  private Telescope telescope;

  public ArmSetBrakeMode (Arm arm, Telescope telescope) {
    this.arm = arm;
    this.telescope = telescope;
    addRequirements(arm, telescope);
  }

  @Override
  public void initialize() {
    if(DriverStation.isDisabled())
    {
      arm.setBrakeMode();
      telescope.setBrakeMode();
    }
  }
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}