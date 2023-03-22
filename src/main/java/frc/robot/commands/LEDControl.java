package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDControlState;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class LEDControl extends CommandBase{
  LED ledSubsystem;
  Limelight limelightSubsystem;
  LEDControlState controlState;
  ColorSensor colorSensorSubsystem;
  Timer flashTimer;
  boolean isFlashing = false;
  public LEDControl(LED LED, Limelight limelight, ColorSensor colorSensor)
  {
    ledSubsystem = LED;
    controlState = LEDControlState.selected;
    limelightSubsystem = limelight;
    colorSensorSubsystem = colorSensor;
    addRequirements(ledSubsystem, limelightSubsystem, colorSensor);
  }
  public void initialize()
  {
    //note to self- add abort state
    switch(controlState)
    {
      case selected:
        if (limelightSubsystem.getTargetVisible()) //or whatever is best used
        {
          flashTimer.start();
          if(!isFlashing)
          {
            ledSubsystem.color(false, true, false);
          }
          }
         
        if(flashTimer.hasElapsed(Constants.FlashTime))
        {
          flashTimer.stop();
          flashTimer.reset();
          ledSubsystem.color(false, false, false);
          /**if(to the left of substation) {
           *  ledSubsystem.color(false, false, true);
           *  controlState = LEDControlState.beginSubstationAlignment;
           * }
           * else if( to the right of substation) {
           *  ledSubsystem.color(true, false, false);
           *  controlState = LEDControlState.beginSubstationAlignment;
           * }
           */
        }
        break;
      case beginSubstationAlignment:
      /*
       * if (aligned with substation) {
          ledSubsystem.color(false, true, false);
          controlState = LEDControlState.substationAligned;
       * }
       */
        break;
      case substationAligned:
     /**if(to the left of substation) {
          ledSubsystem.color(false, false, true);
          controlState = LEDControlState.beginSubstationAlignment;
        }
        else if( to the right of substation) {
          ledSubsystem.color(true, false, false);
          controlState = LEDControlState.beginSubstationAlignment;
        }*/
        if(colorSensorSubsystem.getObject() == "Cube")
        {
          ledSubsystem.color(false, false, false);
          controlState = LEDControlState.transportCube;
        }
        else if(colorSensorSubsystem.getObject() == "Cone") 
        {
          ledSubsystem.color(false, false, false);
          controlState = LEDControlState.transportCone;
        }       
    }
  }

 }
