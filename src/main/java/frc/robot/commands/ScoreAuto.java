package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

public class ScoreAuto extends CommandBase{
   private Drive drive;
   private Arm arm;
   private Claw claw;
   private Limelight limelight;

   private Timer timer = new Timer();
   public ScoreAuto (Drive drive, Arm armSubsystem, Claw clawSubsystem, Limelight limelight) {
    this.drive = drive;
    arm = armSubsystem;
    claw = clawSubsystem;
    this.limelight = limelight;

    addRequirements(arm, claw);
   }

   // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (limelight.getTargetVisible()) {
      
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    arm.stop();
    claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
