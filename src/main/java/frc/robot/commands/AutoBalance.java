package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase{
  private Drive drive;
  private autoBalanceMode currentMode;
  private Timer timer = new Timer();

  public enum autoBalanceMode {
    driving, 
    approaching, //when deg over 9
    past, //when degree falls under 9
    stop, 
    done, 
    abort;
  }

  public AutoBalance(Drive driveSubsystem) {
    drive = driveSubsystem;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    currentMode = autoBalanceMode.driving;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (Constants.driveEnabled) {
      switch (currentMode) {
        case driving:
          drive.drive(0, -Constants.DriveConstants.autoBalanceVelocity, 0);
          if (Math.abs(drive.getPitch()) > Constants.DriveConstants.chargeStationOffAngle) {
            currentMode = autoBalanceMode.approaching;
          }
        case approaching:
          drive.drive(0, -Constants.DriveConstants.autoBalanceVelocity, 0);
          if (Math.abs(drive.getPitch()) < Constants.DriveConstants.chargeStationOffAngle) {
            currentMode = autoBalanceMode.past;
          }
        case past:
          drive.stop();
          // more complicated logic here to readjust roobot after on charging station
          currentMode = autoBalanceMode.stop;
        case stop:
          drive.stop();
          currentMode = autoBalanceMode.done;
        case done: // fall through to break
        case abort:
          break;
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
    return ((currentMode == autoBalanceMode.done) || (currentMode == autoBalanceMode.abort));
  }
}
