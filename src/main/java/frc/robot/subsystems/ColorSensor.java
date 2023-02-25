package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase{

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort); // change port when put sensor on
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kYellowTarget = new Color(0.992, 0.792, 0.329);
  private final Color kPurpleTarget = new Color(0.298, 0.086, 0.761);
  private final Color kOrangeTarget = new Color(1, 0.576, 0.020);

  public ColorSensor() {
    if (Constants.colorSensorEnabled) {
      Color detectedColor = colorSensor.getColor();

      String colorString;
      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

      if (match.color == kOrangeTarget) {
        colorString = "Orange";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else if (match.color == kPurpleTarget) {
        colorString = "Purple";
      } else {
        colorString = "Unknown";
      }

    SmartDashboard.putNumber("Red", detectedColor.red); 
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    }
  }

  public void init() {
    if (Constants.colorSensorEnabled) {
      colorMatcher.addColorMatch(kYellowTarget);
      colorMatcher.addColorMatch(kPurpleTarget);
      colorMatcher.addColorMatch(kOrangeTarget);
    }
  }

  public Color getColor() {
    return colorSensor.getColor();
  }
}
  
