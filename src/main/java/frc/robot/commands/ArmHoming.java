package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmHoming extends CommandBase{
  
  private final Arm arm;

  private final Timer timeout = new Timer();
  private final Timer homeTimer = new Timer();
  private double lastPos;

  public ArmHoming(Arm armSubsystem) {
    arm = armSubsystem;

    addRequirements(arm);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (arm.isHomed()) {
      return;
    }
    timeout.restart();
    homeTimer.restart();
    lastPos = arm.getPosition();
    arm.setLimitSwitch(false);  // hold tight against the rubbery hard stop
    arm.setHoming();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isHomed()) {
      return;
    }
    if (homeTimer.hasElapsed(Constants.ArmConstants.homingNotMovingSec)) {
      double currentPos = arm.getPosition();
      if (lastPos - currentPos < Constants.ArmConstants.homingNotMovingRevs) {
        arm.setPosition(0);
        arm.setHomed();
      } else {
        homeTimer.restart();
        lastPos = currentPos;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    arm.setHomed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (arm.isHomed()) {
      return true;
    }
    if (timeout.hasElapsed(ArmConstants.homingTimeoutSec)) {
      DriverStation.reportError("Arm homing timed out!", false);
      return true;
    }
    return arm.isHomed();
  }
}
