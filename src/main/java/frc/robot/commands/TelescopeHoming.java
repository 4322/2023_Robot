package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

public class TelescopeHoming extends CommandBase{
  
  private final Telescope telescope;

  private final Timer timeout = new Timer();

  public TelescopeHoming(Telescope telescope) {
    this.telescope = telescope;

    addRequirements(telescope);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeout.reset();
    timeout.start();
    //telescope.setLimitSwitch(false);
    telescope.setHoming();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (telescope.getArmSensorPressed() == true) {
      telescope.setPosition(Constants.Telescope.minPosition);
      telescope.setHomed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescope.stop();
    telescope.setHomed();
    //telescope.setLimitSwitch(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return telescope.isHomed() || timeout.hasElapsed(Constants.Telescope.homingTimeoutSec);
  }
}
