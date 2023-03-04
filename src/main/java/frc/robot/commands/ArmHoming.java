package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmHoming extends CommandBase{
  
  private final Arm arm;

  public ArmHoming(Arm armSubsystem) {
    arm = armSubsystem;

    addRequirements(arm);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setCoastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arm.rotateBackward(); TODO: Set homing speed
    if (arm.getArmSensorPressed() == true) {
      arm.setBrakeMode();
      arm.setPosition(ArmConstants.minPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
