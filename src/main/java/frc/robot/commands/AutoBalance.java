package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

// This command currently only works driving backwards onto the charge station.

public class AutoBalance extends CommandBase{
  private Drive drive;
  private autoBalanceMode currentMode;
  private Timer timer = new Timer();

  public enum autoBalanceMode {
    flat, 
    onRamp,
    balanced, 
    abort;
  }

  public AutoBalance(Drive driveSubsystem) {
    drive = driveSubsystem;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    currentMode = autoBalanceMode.flat;
    drive.drive(-Constants.DriveConstants.autoBalanceFlatPower, 0, 0);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (Constants.driveEnabled) {
      if (timer.hasElapsed(Constants.DriveConstants.autoBalanceTimeoutSec)) {
        currentMode = autoBalanceMode.abort;
      }
      switch (currentMode) {
        case flat:
          if (Math.abs(drive.getPitch()) > Constants.DriveConstants.chargeStationTiltedMinDeg) {
            drive.drive(-Constants.DriveConstants.autoBalanceRampPower, 0, 0);
            currentMode = autoBalanceMode.onRamp;
          }
          break;
        case onRamp:
          if (Math.abs(drive.getPitch()) < Constants.DriveConstants.chargeStationBalancedMaxDeg) {
            drive.stop();
            currentMode = autoBalanceMode.balanced;
          }
          break;
        case balanced: // fall through to break
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
    return ((currentMode == autoBalanceMode.balanced) || (currentMode == autoBalanceMode.abort));
  }
}
