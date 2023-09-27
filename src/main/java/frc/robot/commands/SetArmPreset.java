package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class SetArmPreset extends InstantCommand {
  private ArmMove.Position pos;
  private DriveManual driveAutoPose;
  private DriveManual driveDefault;

  // all parameters
  public SetArmPreset(Drive drive, ArmMove.Position pos) {
    this.pos = pos;
    if (driveAutoPose == null) {
      // reuse auto-pose commands so they won't restart if the operator 
      // presses the same button twice
      driveAutoPose = new DriveManual(drive, DriveManual.AutoPose.usePreset);
      driveDefault = new DriveManual(drive, DriveManual.AutoPose.none);
    }
    // don't interrupt existing arm commands because the trigger command won't restart
  }

  @Override
  public void initialize() {

    // check if inside the yellow box of allowed operator actions on the state diagram
    if (ArmMove.isInBot()) {
      ArmMove.setArmPreset(pos);

      // switch to new auto-pos, if needed
      if (DriveManual.isScoreAutoPoseActive()
          && (pos == ArmMove.Position.scoreLow
          || pos == ArmMove.Position.scoreMid
          || pos == ArmMove.Position.scoreHigh)) {
        driveAutoPose.schedule();  // does nothing if already scheduled
      } else if (DriveManual.isScoreAutoPoseActive()) {
        // break out of score auto pose so the next button press to
        // activate load auto-pose will work
        driveDefault.schedule();
      } else if (DriveManual.isLoadAutoPoseActive()) {
        // break out of load auto pose so the next button press to
        // activate score auto-pose will work
        driveDefault.schedule();
      }
    // check if this is an allowed state transition outside the yellow box
    } else if (((pos == ArmMove.Position.scoreMid) || (pos == ArmMove.Position.scoreHigh))
               && ((ArmMove.getArmPreset() == ArmMove.Position.scoreMid) ||
                   (ArmMove.getArmPreset() == ArmMove.Position.scoreHigh))) {
      ArmMove.setArmPreset(pos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
