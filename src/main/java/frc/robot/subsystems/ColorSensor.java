package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {
  private ShuffleboardTab tab;
  private GenericEntry Red;
  private GenericEntry Green;
  private GenericEntry Blue;
  private GenericEntry Confidence;
  private GenericEntry DetectedObject;
  private String ObjectString;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort); // change port when put
                                                                        // sensor on
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kYellowTarget = new Color(0.992, 0.792, 0.329);
  private final Color kPurpleTarget = new Color(0.298, 0.086, 0.761);
  private final Color kOrangeTarget = new Color(1, 0.576, 0.020);
  private ColorMatchResult match;
  private Color detectedColor;
  public ColorSensor() {
    if (Constants.colorSensorEnabled) {
      colorMatcher.addColorMatch(kYellowTarget);
      colorMatcher.addColorMatch(kPurpleTarget);
      colorMatcher.addColorMatch(kOrangeTarget);
      
     
    }
  }

  public void init() {
    if (Constants.colorSensorEnabled) {
      detectedColor = colorSensor.getColor();

      match = colorMatcher.matchClosestColor(detectedColor);

      if (match.color == kOrangeTarget) {
        ObjectString = "Nothing";
      } else if (match.color == kYellowTarget) {
        ObjectString = "Cone";
      } else if (match.color == kPurpleTarget) {
        ObjectString = "Cube";
      }


      if (Constants.debug) {
        tab = Shuffleboard.getTab("Detected Object");
        Red = tab.add("Red Value", detectedColor.red).getEntry();
        Green = tab.add("Green Value", detectedColor.green).getEntry();
        Blue = tab.add("Blue Value", detectedColor.blue).getEntry();
        Confidence = tab.add("Confidence", match.confidence).getEntry();
        DetectedObject = tab.add("Detected Object", ObjectString).getEntry();
      }
    }
  }

  private Color getColor() {
    if (Constants.colorSensorEnabled) {
      return colorSensor.getColor();
    } else {
      return null;
    }
  }

  public String getObject() {
    if (Constants.colorSensorEnabled) {
      detectedColor = colorSensor.getColor();

      match = colorMatcher.matchClosestColor(detectedColor);
      if (match.color == kOrangeTarget) {
        ObjectString = "Nothing";
      } else if (match.color == kYellowTarget) {
        ObjectString = "Cone";
      } else if (match.color == kPurpleTarget) {
        ObjectString = "Cube";
      }


      if (Constants.debug) {
        tab = Shuffleboard.getTab("Detected Object");
        Red = tab.add("Red Value", detectedColor.red).getEntry();
        Green = tab.add("Green Value", detectedColor.green).getEntry();
        Blue = tab.add("Blue Value", detectedColor.blue).getEntry();
        Confidence = tab.add("Confidence", match.confidence).getEntry();
        DetectedObject = tab.add("Detected Object", ObjectString).getEntry();
      }
      return ObjectString;
    } else {
      return null;
    }
  }
}

