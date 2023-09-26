package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;

public class SetArmPreset extends InstantCommand {
  private ArmMove.Position pos;
  private DriveManual driveAutoPos;

  // all parameters
  public SetArmPreset(Drive drive, ArmMove.Position pos) {
    this.pos = pos;
    if (driveAutoPos == null) {
      // reuse a single auto-pose command so it won't restart if the operator 
      // presses the same button twice
      driveAutoPos = new DriveManual(drive, DriveManual.AutoPose.usePreset);
    }
    
    // don't interrupt existing arm commands because the trigger command won't restart
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ArmMove.isInBot()) {
      ArmMove.setArmPreset(pos);

      // switch to new auto-pos, if needed
      if (DriveManual.isScoringAutoPoseActive()
          && (pos == ArmMove.Position.scoreLow
          || pos == ArmMove.Position.scoreMid
          || pos == ArmMove.Position.scoreHigh)) {
        driveAutoPos.schedule();  // does nothing if already scheduled
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
