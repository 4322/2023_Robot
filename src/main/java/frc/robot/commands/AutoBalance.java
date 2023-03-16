package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {
  private Drive drive;
  private autoBalanceMode currentMode;
  private Timer timer = new Timer();
  private Timer debounceTimer = new Timer();
  private int driveSign;

  public enum autoBalanceMode {
    flat, onRamp, balanced, finished, abort, adjusting;
  }

  public AutoBalance(Drive driveSubsystem, boolean forward) {
    drive = driveSubsystem;
    if (forward) {
      driveSign = 1;
    } else {
      driveSign = -1;
    }

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    currentMode = autoBalanceMode.flat;
    drive.drive(driveSign * Constants.DriveConstants.autoBalanceFlatPower, 0, 0);
    timer.reset();
    timer.start();
    debounceTimer.reset();
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
            debounceTimer.start();
          } else {
            debounceTimer.reset();
            debounceTimer.stop();
          }
          if (debounceTimer.hasElapsed(Constants.DriveConstants.DebounceTime)) {
            drive.drive(driveSign * Constants.DriveConstants.autoBalanceRampPower, 0, 0);
            currentMode = autoBalanceMode.onRamp;
            debounceTimer.reset();
            debounceTimer.stop();
          }
          break;
        case onRamp:
          if (Math.abs(drive.getPitch()) < Constants.DriveConstants.chargeStationBalancedMaxDeg) {
            debounceTimer.start();
          }
          else {
            debounceTimer.reset();
            debounceTimer.stop();
          }
          if (debounceTimer.hasElapsed(Constants.DriveConstants.DebounceTime)) {
            drive.stop();
            currentMode = autoBalanceMode.balanced;
            debounceTimer.reset();
            debounceTimer.stop();
          }
          break;
        case balanced: // fall through to break
          if ((Math.abs(drive.getPitch())) <= Constants.DriveConstants.LevelChargeStationDeg) {
            debounceTimer.start();
          } else if ((drive.getPitch() >= Constants.DriveConstants.LevelChargeStationDeg)) {
            debounceTimer.reset();
            debounceTimer.stop();
            drive.drive(Constants.DriveConstants.autoBalanceAdjustment, 0, 0);
            currentMode = autoBalanceMode.adjusting;
          } else {
            debounceTimer.reset();
            debounceTimer.stop();
            drive.drive(-Constants.DriveConstants.autoBalanceAdjustment, 0, 0);
            currentMode = autoBalanceMode.adjusting;

          }
          if (debounceTimer.hasElapsed(Constants.DriveConstants.DebounceTime)) {
            currentMode = autoBalanceMode.finished;
            drive.drive(Constants.DriveConstants.autoBalanceAdjustment, 0, 0);
          }
          break;

        case adjusting:
          if ((Math.abs(drive.getPitch())) <= Constants.DriveConstants.LevelChargeStationDeg)
          {
            drive.stop();
            debounceTimer.start();
            currentMode = autoBalanceMode.balanced;
          }
        case finished:
          break;
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
