package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawOuttake extends CommandBase {
  private Claw claw;
  private boolean outtakeStarted;
  private boolean force;

  public ClawOuttake(boolean force) {
    claw = Claw.getInstance();
    this.force = force;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    outtakeStarted = false;
  }

  @Override
  public void execute() {
    if (!outtakeStarted && (ArmMove.isSafeToOuttake() || force)) {
      claw.changeState(Claw.ClawMode.outtaking);
      outtakeStarted = true;
    }
    // don't change preset if outtake jams
    if (claw.isOuttakeUpToSpeed()) {
      ArmMove.setArmPreset(ArmMove.Position.loadSingleExtend);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (outtakeStarted) {
      claw.changeState(Claw.ClawMode.stopped);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
