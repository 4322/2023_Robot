package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class TimedClawOuttake extends CommandBase {
  private Claw claw;

  private double timeLimit;
  private Timer timer = new Timer();

  public TimedClawOuttake(Claw clawSubsystem, double time) {
    claw = clawSubsystem;
    timeLimit = time;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.changeState(Claw.ClawMode.outtaking);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.changeState(Claw.ClawMode.stopped);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeLimit);
  }
}
