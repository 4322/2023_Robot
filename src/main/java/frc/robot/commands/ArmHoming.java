package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmHoming extends CommandBase{
  
  private final Arm arm;

  private final Timer timeout = new Timer();

  public ArmHoming(Arm armSubsystem) {
    arm = armSubsystem;

    addRequirements(arm);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setLimitSwitch(false);
    arm.setHoming();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getArmSensorPressed() == true) {
      arm.setPosition(ArmConstants.minPosition);
      arm.setHomed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    arm.setHomed();
    arm.setLimitSwitch(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isHomed() || timeout.hasElapsed(ArmConstants.homingTimeout);
  }
}
