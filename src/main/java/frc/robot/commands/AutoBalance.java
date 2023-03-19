package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.utility.OrangeMath;

public class AutoBalance extends CommandBase {
  private Drive drive;
  private autoBalanceMode currentMode;
  private Timer abortTimer = new Timer();
  private Timer debounceTimer = new Timer();
  private int driveSign;
  private double poseDeg;
  private double poseSign;

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
    drive.resetRotatePID();
    abortTimer.reset();
    abortTimer.start();
    debounceTimer.reset();
    debounceTimer.stop();

    // align to charge station by rotating the least amount
    if (Math.abs(OrangeMath.boundDegrees(drive.getAngle())) <= 90) {
      poseDeg = 0;
      poseSign = 1;
    } else {
      poseDeg = 180;
      poseSign = -1;
    }
  }

  @Override
  public void execute() {
    if (Constants.driveEnabled) {
      if (abortTimer.hasElapsed(Constants.DriveConstants.autoBalanceTimeoutSec)) {
        currentMode = autoBalanceMode.abort;
      }
      switch (currentMode) {
        case flat:
          if (Math.abs(drive.getPitch()) < Constants.DriveConstants.chargeStationTiltedMinDeg) {
            // need to keep making the drive call to maintain heading
            drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceFlatPower, 0, poseDeg,
                Constants.DriveConstants.manualRotateToleranceDegrees);
          } else {
            currentMode = autoBalanceMode.onRamp;
          }
          break;
        case onRamp:
          if (Math.abs(drive.getPitch()) > Constants.DriveConstants.chargeStationDroppingDeg) {
            // need to keep making the drive call to maintain heading
            drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceRampPower, 0, poseDeg,
                Constants.DriveConstants.manualRotateToleranceDegrees);
          } else {
            drive.stop();
            currentMode = autoBalanceMode.balanced;
            debounceTimer.start();
          }
          break;
        case balanced:
          if (Math.abs(drive.getPitch()) <= Constants.DriveConstants.chargeStationBalancedMaxDeg) {
            debounceTimer.start();
          } else {
            debounceTimer.reset();
            debounceTimer.stop();
            currentMode = autoBalanceMode.adjusting;
          }
          if (debounceTimer.hasElapsed(Constants.DriveConstants.debounceSec)) {
            currentMode = autoBalanceMode.finished;
          }
          break;
        case adjusting:
          if ((Math.abs(drive.getPitch())) <= Constants.DriveConstants.chargeStationBalancedMaxDeg) {
            drive.stop();
            debounceTimer.start();
            currentMode = autoBalanceMode.balanced;
          } else {
            // need to keep making the drive call to maintain heading
            drive.driveAutoRotate(
                poseSign * Math.copySign(Constants.DriveConstants.autoBalanceAdjustmentPower, drive.getPitch()), 0,
                poseDeg,
                Constants.DriveConstants.manualRotateToleranceDegrees);
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
    if (Constants.debug) {
      DataLogManager.log("Auto Balance Time: " + abortTimer.get());
    }
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return ((currentMode == autoBalanceMode.finished) || (currentMode == autoBalanceMode.abort));
  }
}
