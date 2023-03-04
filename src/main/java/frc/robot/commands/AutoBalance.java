package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase{
  private Drive drive;
  String direction;
  private autoBalanceMode currentMode;

  public enum autoBalanceMode {
    driving, 
    approaching, //when deg over 9
    past, //when degree falls under 9
    stop, 
    done, 
    abort;
  }

  public AutoBalance(Drive driveSubsystem, String direction) {
    drive = driveSubsystem;
    this.direction = direction;

    addRequirements(drive);

    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Constants.driveEnabled) {
      switch (currentMode) {
        case driving:
        case approaching:
        case past:
        case stop:
        case done:
        case abort:
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
