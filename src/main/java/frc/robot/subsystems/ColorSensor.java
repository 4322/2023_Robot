package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase{

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kYellowTarget = new Color(0.992, 0.792, 0.329);
  private final Color kPurpleTarget = new Color(0.298, 0.086, 0.761);
  private final Color kOrangeTarget = new Color(1, 0.576, 0.020);
  public ColorSensor() {
    
  }

  public void init() {
    colorMatcher.addColorMatch(kYellowTarget);
    colorMatcher.addColorMatch(kPurpleTarget);
    colorMatcher.addColorMatch(kOrangeTarget);
  }

  public Color getYellow() {
    return kYellowTarget;
  }

  public Color getPurple() {
    return kPurpleTarget;
  }

  public Color getOrange() {
    return kOrangeTarget;
  }
}
  
