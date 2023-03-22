package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDControlState;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

public class LEDControl extends CommandBase {
  LED ledSubsystem;
  Limelight limelightFrontSubsystem;
  Limelight limelightBackSubsystem;
  LEDControlState controlState;
  ColorSensor colorSensorSubsystem;
  Timer flashTimer;
  boolean isFlashing = false;
  double limelightY;

  public LEDControl(LED LED, Limelight limelightFront, Limelight limelightBack,ColorSensor colorSensor) {
    if (Constants.ledEnabled && Constants.limeLightEnabled && Constants.colorSensorEnabled) {
      ledSubsystem = LED;
      controlState = LEDControlState.selected;
      limelightFrontSubsystem = limelightFront;
      limelightBackSubsystem = limelightBack;
      colorSensorSubsystem = colorSensor;
      addRequirements(ledSubsystem, limelightFrontSubsystem, limelightBackSubsystem, colorSensor);
    }
    else
    {
      controlState = LEDControlState.abort;
    }
  }

  @Override
  public void initialize() {
    if (Constants.ledEnabled && Constants.limeLightEnabled)
    {
      limelightFrontSubsystem.switchPipeline(2);
    }
  }

  @Override
  public void execute() {
    
    // note to self- add abort state
    switch (controlState) {
      case selected:
        if (limelightFrontSubsystem.getTargetVisible()) // or whatever is best used
        {
          limelightY = limelightFrontSubsystem.getTargetPosRobotRelative().getY();
          ledSubsystem.color(false, false, false);
          if (limelightY < -Constants.AlignmentTolerance) {
            ledSubsystem.color(false, false, true);
            controlState = LEDControlState.beginSubstationAlignment;
          } else if (limelightY > Constants.AlignmentTolerance) {
            ledSubsystem.color(true, false, false);
            controlState = LEDControlState.beginSubstationAlignment;
          } else {
            ledSubsystem.color(false, true, false);
            controlState = LEDControlState.substationAligned;
          }
        }        
        break;
      case beginSubstationAlignment:
        limelightY = limelightFrontSubsystem.getTargetPosRobotRelative().getY();
        if ((limelightY > -Constants.AlignmentTolerance)
            && (limelightY < Constants.AlignmentTolerance)) {
          ledSubsystem.color(false, true, false);
          controlState = LEDControlState.substationAligned;
        }
        break;
      case substationAligned:
        limelightY = limelightFrontSubsystem.getTargetPosRobotRelative().getY();
        if (limelightY < -Constants.AlignmentTolerance) {
          ledSubsystem.color(false, false, true);
          controlState = LEDControlState.beginSubstationAlignment;
        } else if (limelightY > Constants.AlignmentTolerance) {
          ledSubsystem.color(true, false, false);
          controlState = LEDControlState.beginSubstationAlignment;
        }
        if (colorSensorSubsystem.getObject() == "Cube") {
          ledSubsystem.color(false, false, false);
          controlState = LEDControlState.transportObject;
          limelightBackSubsystem.switchPipeline(1);
        } else if (colorSensorSubsystem.getObject() == "Cone") {
          ledSubsystem.color(false, false, false);
          controlState = LEDControlState.transportObject;
          limelightBackSubsystem.switchPipeline(0);
        }
        break;
      case transportObject:
        if (limelightBackSubsystem.getTargetVisible()) {
          limelightY = limelightBackSubsystem.getTargetPosRobotRelative().getY();
          ledSubsystem.color(false, false, false);
          if (limelightY < -Constants.AlignmentTolerance) {
            ledSubsystem.color(false, false, true);
            controlState = LEDControlState.beginGridAlignment;
          } else if (limelightY > Constants.AlignmentTolerance) {
            ledSubsystem.color(true, false, false);
            controlState = LEDControlState.beginGridAlignment;
          } else {
            ledSubsystem.color(false, true, false);
            controlState = LEDControlState.gridAligned;
          }
        }
        break;
      case beginGridAlignment:
        limelightY = limelightBackSubsystem.getTargetPosRobotRelative().getY();
        if ((limelightY > -Constants.AlignmentTolerance)
            && (limelightY < Constants.AlignmentTolerance)) {
          ledSubsystem.color(false, true, false);
          controlState = LEDControlState.gridAligned;
        }
        break;
      case gridAligned:
        limelightY = limelightBackSubsystem.getTargetPosRobotRelative().getY();
        if (limelightY < -Constants.AlignmentTolerance) {
          ledSubsystem.color(false, false, true);
          controlState = LEDControlState.beginGridAlignment;
        } else if (limelightY > Constants.AlignmentTolerance) {
          ledSubsystem.color(true, false, false);
          controlState = LEDControlState.beginGridAlignment;
        }
        if(colorSensorSubsystem.getObject() == "Nothing")
        {
          controlState = LEDControlState.done;
        }
        break;
      case abort:
      case done: 
        break;
    }

  }
  @Override
  public boolean isFinished() {
    return ((controlState == LEDControlState.done));
  }
}
