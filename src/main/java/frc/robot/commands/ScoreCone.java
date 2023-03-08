package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawMode;

public class ScoreCone extends CommandBase{
  private Arm arm;
  private Claw claw;
  private scoreMode currentMode;
  private Timer timer = new Timer();

  public enum scoreMode {
    rotating,
    atPosition,
    done,
    abort;
  }

  public ScoreCone(Arm armSubsystem, Claw clawSubsystem) {
    arm = armSubsystem;
    claw = clawSubsystem;

    addRequirements(arm, claw);
  }

  @Override
  public void initialize() {
    currentMode = scoreMode.rotating;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    switch (currentMode) {
      case rotating:
        if (Constants.armEnabled) {
          arm.rotateToPosition(Constants.ArmConstants.MidScoringPosition);
          if (arm.isAtTarget()) {
            currentMode = scoreMode.atPosition;
          }
        } else {
          currentMode = scoreMode.done;
        }
      case atPosition:
        if (Constants.clawEnabled) {
          claw.changeState(ClawMode.outtaking);
        }

        if (timer.hasElapsed(1)) {
          currentMode = scoreMode.done;
        }
      case done: // falls to break
      case abort:
        break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    claw.changeState(ClawMode.stopped);
    currentMode = scoreMode.done;
  }

  @Override
  public boolean isFinished() {
    return ((currentMode == scoreMode.done) || (currentMode == scoreMode.abort));
  }
}
