package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Telescope;

public class TelescopeHoming extends CommandBase{
  
  private final Telescope telescope;

  private final Timer timeout = new Timer();
  private final Timer homeTimer = new Timer();
  private double lastPos;
  private boolean force;

  public TelescopeHoming(Telescope telescope, boolean force) {
    this.telescope = telescope;
    this.force = force;

    addRequirements(telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!force && telescope.isHomed()) {
      return;
    }
    timeout.restart();
    homeTimer.restart();
    lastPos = telescope.getPosition();
    telescope.setHoming();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (telescope.isHomed()) {
      return;
    }
    if (homeTimer.hasElapsed(Constants.Telescope.notMovingSec)) {
      double currentPos = telescope.getPosition();
      if (lastPos - currentPos < Constants.Telescope.notMovingRevs) {
        telescope.setPosition(0);
        telescope.setHomed();
      } else {
        homeTimer.restart();
        lastPos = currentPos;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescope.stop();
    telescope.setHomed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (telescope.isHomed()) {
      return true;
    }
    if (timeout.hasElapsed(Constants.Telescope.homingTimeoutSec)) {
      DriverStation.reportError("Telescope homing timed out!", false);
      return true;
    }
    return telescope.isHomed();
  }
}
