package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDControlState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class LEDControl extends CommandBase{
  LED ledSubsystem;
  Limelight limelightSubsystem;
  LEDControlState controlState;
  Timer flashTimer;
  public LEDControl(LED LED, Limelight limelight)
  {
    ledSubsystem = LED;
    controlState = LEDControlState.selected;
    limelightSubsystem = limelight;
    addRequirements(ledSubsystem, limelightSubsystem);
  }
  public void initialize()
  {
    //note to self- add abort state
    switch(controlState)
    {
      case selected:
        if (limelightSubsystem.getTargetVisible())
        {
          flashTimer.start();
          ledSubsystem.color(false, true, false);
        }
        if(flashTimer.hasElapsed(Constants.FlashTime))
        {
          flashTimer.stop();
          flashTimer.reset();
          ledSubsystem.color(false, false, false);
          /**if( to the left of substation)
           * {
           *  ledSubsystem.color(false, false, true);
           *  controlState = LEDControlState.LeftOfSubstation;
           * }
           * else if( to the right of substation)
           * {
           *  ledSubsystem.color(true, false, false);
           *  controlState = LEDControlState.RightOfSubtraction;
           * }
           *
           */
        }
        break;
      case LeftOfSubstation:
        
      case RightofSubstation:
    }
  }

 }
