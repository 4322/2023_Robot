package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {
  private Drive drive;
  private autoBalanceMode currentMode;
  private Timer abortTimer = new Timer();
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
    drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceFlatPower, 0, 0, Constants.DriveConstants.manualRotateToleranceDegrees);
    abortTimer.reset();
    abortTimer.start();
    debounceTimer.reset();
  }

  @Override
  public void execute() {
    if (Constants.driveEnabled) {
      if (abortTimer.hasElapsed(Constants.DriveConstants.autoBalanceTimeoutSec)) {
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
            drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceRampPower, 0, 0,Constants.DriveConstants.manualRotateToleranceDegrees);
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
        case balanced:
          if ((Math.abs(drive.getPitch())) <= Constants.DriveConstants.LevelChargeStationDeg) {
            debounceTimer.start();
          } else if ((drive.getPitch() >= Constants.DriveConstants.LevelChargeStationDeg)) {
            debounceTimer.reset();
            debounceTimer.stop();
            drive.driveAutoRotate(Constants.DriveConstants.autoBalanceAdjustment, 0, 0,Constants.DriveConstants.manualRotateToleranceDegrees);
            currentMode = autoBalanceMode.adjusting;
          } else {
            debounceTimer.reset();
            debounceTimer.stop();
            drive.driveAutoRotate(-Constants.DriveConstants.autoBalanceAdjustment, 0, 0,Constants.DriveConstants.manualRotateToleranceDegrees);
            currentMode = autoBalanceMode.adjusting;

          }
          if (debounceTimer.hasElapsed(Constants.DriveConstants.DebounceTime)) {
            currentMode = autoBalanceMode.finished;
          }
          break;

        case adjusting:
          if ((Math.abs(drive.getPitch())) <= Constants.DriveConstants.LevelChargeStationDeg)
          {
            drive.stop();
            debounceTimer.start();
            currentMode = autoBalanceMode.balanced;
          }
          break;
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
    return ((currentMode == autoBalanceMode.finished) || (currentMode == autoBalanceMode.abort));
  }
}
