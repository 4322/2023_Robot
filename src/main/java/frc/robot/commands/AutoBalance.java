package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
import frc.utility.OrangeMath;

public class AutoBalance extends CommandBase {
  private Drive drive;
  private autoBalanceMode currentMode;
  private Timer abortTimer = new Timer();
  private Timer rampTimer = new Timer();
  private Timer debounceTimer = new Timer();
  private int driveSign;
  private double pitch;
  private double absPitch;
  private double maxAbsPitch;
  private double startAdjustPitch;
  private double poseDeg;
  private double poseSign;
  private boolean autonomous;

  public enum autoBalanceMode {
    flat, onRamp, dropping, leveling, finished, abort, adjusting;
  }

  public AutoBalance(Drive driveSubsystem, boolean forward, boolean autonomous) {
    drive = driveSubsystem;
    if (forward) {
      driveSign = 1;
    } else {
      driveSign = -1;
    }
    this.autonomous = autonomous;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    currentMode = autoBalanceMode.flat;
    drive.resetRotatePID();
    maxAbsPitch = 0;
    abortTimer.restart();
    rampTimer.reset();
    rampTimer.stop();

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
      if (abortTimer.hasElapsed(Constants.DriveConstants.autoBalanceTimeoutSec)
          || ((abortTimer.hasElapsed(Constants.DriveConstants.autoBalanceFlatTimeoutSec)
              && currentMode == autoBalanceMode.flat))) {
        currentMode = autoBalanceMode.abort;
      }
      pitch = drive.getPitch();
      absPitch = Math.abs(pitch);
      if (rampTimer.hasElapsed(Constants.DriveConstants.rampImpulseSec) && absPitch > maxAbsPitch) {
        maxAbsPitch = absPitch;
      }
      switch (currentMode) {
        case flat:
          if (absPitch < Constants.DriveConstants.chargeStationTiltedMinDeg) {
            // need to keep making the drive call to maintain heading
            drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceFlatPower, 0, poseDeg,
                Constants.DriveConstants.Auto.rotateToleranceDegrees);
          } else {
            currentMode = autoBalanceMode.onRamp;
            rampTimer.start();
            drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceRampPower, 0, poseDeg,
            Constants.DriveConstants.Auto.rotateToleranceDegrees);
          }
          break;
        case onRamp:
          if ((absPitch <= (maxAbsPitch - Constants.DriveConstants.chargeStationDroppingDeg)) ||
              ((driveSign * poseSign > 0) && (pitch <= Constants.DriveConstants.chargeStationBalancedMaxDeg)) ||
              ((driveSign * poseSign < 0) && (pitch >= -Constants.DriveConstants.chargeStationBalancedMaxDeg))) {
            // charge station has started to drop, give it a chance to level out
            drive.stop();
            debounceTimer.restart();
            currentMode = autoBalanceMode.dropping;
            if (Constants.debug) {
              DriverStation.reportWarning("maxAbsPitch: " + maxAbsPitch + ", dropAbsPitch: " + absPitch, false);
            }
          } else {
            // need to keep making the drive call to maintain heading
            drive.driveAutoRotate(driveSign * Constants.DriveConstants.autoBalanceRampPower, 0, poseDeg,
                Constants.DriveConstants.Auto.rotateToleranceDegrees);
          }
          break;
        case dropping:
          if (absPitch <= Constants.DriveConstants.chargeStationBalancedMaxDeg) {
            debounceTimer.restart();
            currentMode = autoBalanceMode.leveling;
          } else if (debounceTimer.hasElapsed(Constants.DriveConstants.droppingSec)) {
            startAdjustPitch = pitch;
            currentMode = autoBalanceMode.adjusting;
          }
          break;
        case leveling:
          if (debounceTimer.hasElapsed(Constants.DriveConstants.levelingSec)) {
            if (absPitch <= Constants.DriveConstants.chargeStationBalancedMaxDeg) {
              currentMode = autoBalanceMode.finished;
              LED.getInstance().setPresetAccepted();
            } else {
              startAdjustPitch = pitch;
              currentMode = autoBalanceMode.adjusting;
            }
          }
          break;
        case adjusting:
          if ((absPitch <= Constants.DriveConstants.chargeStationBalancedMaxDeg)
              || ((startAdjustPitch >= 0)
                  && (pitch < startAdjustPitch - Constants.DriveConstants.chargeStationDroppingDeg))
              || ((startAdjustPitch < 0) && (pitch > startAdjustPitch
                  + Constants.DriveConstants.chargeStationDroppingDeg))) {
            drive.stop();
            debounceTimer.restart();
            currentMode = autoBalanceMode.dropping;
          } else {
            // need to keep making the drive call to maintain heading
            drive.driveAutoRotate(
                poseSign * Math.copySign(Constants.DriveConstants.autoBalanceAdjustmentPower, pitch), 0,
                poseDeg,
                Constants.DriveConstants.Auto.rotateToleranceDegrees);
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
      DriverStation.reportWarning("Auto Balance Time: " + abortTimer.get(), false);
    }
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    if(autonomous && (DriverStation.getMatchTime() > 0.0) && (DriverStation.getMatchTime() <= 0.1))
    {
      return true;
    }
    return ((currentMode == autoBalanceMode.finished) || (currentMode == autoBalanceMode.abort));
  }
}
