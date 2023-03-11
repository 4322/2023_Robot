package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
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
    setRotatingForward,
    rotatingForward,
    atPosition,
    outtaking,
    setRotatingBack,
    rotatingBack,
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
    currentMode = scoreMode.rotatingForward;
    timer.reset();
    timer.stop();
  }

  @Override
  public void execute() {
    switch (currentMode) {
      case setRotatingForward:
        if (!arm.rotateToPosition(Constants.ArmConstants.MidScoringPosition)) {
          currentMode = scoreMode.abort;
        } else {
          currentMode = scoreMode.rotatingForward;
        }
        break;
      case rotatingForward:
        if (arm.isAtTarget()) {
          currentMode = scoreMode.atPosition;
        }
        break;
      case atPosition:
        if (!claw.changeState(ClawMode.outtaking)) {
          currentMode = scoreMode.abort;
        } else {
          currentMode = scoreMode.outtaking;
          timer.start();
        }
        break;
      case outtaking:
        if (timer.hasElapsed(0.5)) {
          currentMode = scoreMode.setRotatingBack;
        }
        break;
      case setRotatingBack:
        if (!arm.rotateToPosition(Constants.ArmConstants.LoadPosition)) {
          currentMode = scoreMode.abort;
        } else {
          currentMode = scoreMode.rotatingBack;
        }
        break;
      case rotatingBack:
        if (arm.isAtTarget()) {
          currentMode = scoreMode.done;
        }
        break;
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
    if (currentMode == scoreMode.abort) {
      DataLogManager.log("Aborted Piece Score");
    }
  }

  @Override
  public boolean isFinished() {
    return ((currentMode == scoreMode.done) || (currentMode == scoreMode.abort));
  }
}
