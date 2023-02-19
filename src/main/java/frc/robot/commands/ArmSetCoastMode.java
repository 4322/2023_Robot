package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmSetCoastMode extends CommandBase{
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;  // allows arm to coast after match
  }
}
