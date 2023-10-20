package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

// Drive over the charge station forward.

public class AutoDriveOverCharge extends CommandBase{
  private Drive drive;
  private autoDriveOverChargeMode currentMode;
  private Timer commandTimer = new Timer();
  private Timer flatTimer = new Timer();

  public enum autoDriveOverChargeMode {
    flatBeforeRamp, 
    upRamp,
    downRamp, 
    flatAfterRamp,
    done,
    abort;
  }

  public AutoDriveOverCharge(Drive driveSubsystem) {
    drive = driveSubsystem;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    currentMode = autoDriveOverChargeMode.flatBeforeRamp;
    drive.drive(Constants.DriveConstants.autoBalanceFlatPower, 0, 0);
    commandTimer.reset();
    commandTimer.start();
  }

  @Override
  public void execute() {
    if (Constants.driveEnabled) {
      if (commandTimer.hasElapsed(Constants.DriveConstants.autoDriveOverChargeTimeoutSec)) {
        currentMode = autoDriveOverChargeMode.abort;
      }
      switch (currentMode) {
        case flatBeforeRamp:
          drive.driveAutoRotate(Constants.DriveConstants.autoBalanceFlatPower, 0, 0);
          if (drive.getPitch() > Constants.DriveConstants.chargeStationTiltedMinDeg) {
           
            currentMode = autoDriveOverChargeMode.upRamp;
          }
          break;
        case upRamp:
          drive.driveAutoRotate(Constants.DriveConstants.autoChargePower, 0, 0);
          if (drive.getPitch() < -Constants.DriveConstants.chargeStationTiltedMinDeg) {
            currentMode = autoDriveOverChargeMode.downRamp;
          }
          break;
        case downRamp: 
          drive.driveAutoRotate(Constants.DriveConstants.autoChargePower, 0, 0);
          if (drive.getPitch() > -Constants.DriveConstants.autoDriveOverChargeFlatMaxDeg) {
            flatTimer.reset();
            flatTimer.start();
            currentMode = autoDriveOverChargeMode.flatAfterRamp;
          }
          break;
        case flatAfterRamp:
          drive.driveAutoRotate(Constants.DriveConstants.autoChargePower, 0, 0);
          if (Math.abs(drive.getPitch()) < 1) {
            drive.stop();
            currentMode = autoDriveOverChargeMode.done;
          }
          break;
        case done:
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
    return ((currentMode == autoDriveOverChargeMode.done) || (currentMode == autoDriveOverChargeMode.abort));
  }
}
