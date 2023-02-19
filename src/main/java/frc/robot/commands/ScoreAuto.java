package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.ClawOuttake;

public class ScoreAuto extends CommandBase {
  private scoringStates currentMode;
  private Drive drive;
  private Arm arm;
  private Claw claw;
  private Limelight limelight;

  private Timer timer = new Timer();

  public enum scoringStates {
    firstStop, calibrate, score, done, abort;
  }

  public ScoreAuto(Drive drive, Arm armSubsystem, Claw clawSubsystem, Limelight limelight) {
    this.drive = drive;
    arm = armSubsystem;
    claw = clawSubsystem;
    this.limelight = limelight;

    addRequirements(arm, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentMode = scoringStates.firstStop;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentMode) {
      case firstStop:
        drive.stop();
        currentMode = scoringStates.calibrate;
      case calibrate:
      if (currentMode == scoringStates.calibrate){
        // add path planner calculations
        // need to change drivex, drivey, and rotate variables
        drive.drive(Constants.DriveConstants.driveX, Constants.DriveConstants.driveY,
        Constants.DriveConstants.rotate);
        if (limelight.getDistanceInches() == 0){
          currentMode = scoringStates.score;
        }
      }
      case score:
        // run claw outtake
        currentMode = scoringStates.done;
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
