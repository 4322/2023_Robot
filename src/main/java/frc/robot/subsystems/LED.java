package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants;

/**
 * Offers basic control of RGB (three-color) LEDs, given a PCM ID as well as RGB ports
 */
public class LED extends SubsystemBase {
  LEDStrip led1;
  LEDStrip led2;
  Solenoid power1;
  Solenoid power2;
  private LEDColor ledColor;

  public LED() {
    if (Constants.ledEnabled) {
      power1 = new Solenoid(LEDConstants.pcmID, PneumaticsModuleType.REVPH, LEDConstants.pPort1);
      power1.set(true);
      power2 = new Solenoid(LEDConstants.pcmID, PneumaticsModuleType.REVPH, LEDConstants.pPort2);
      power2.set(true);
      led1 = new LEDStrip(LEDConstants.pcmID, LEDConstants.rPort1, LEDConstants.gPort1,
          LEDConstants.bPort1);
      led2 = new LEDStrip(LEDConstants.pcmID, LEDConstants.rPort2, LEDConstants.gPort2,
          LEDConstants.bPort2);
      ledColor = LEDColor.none;
    }
  }

  public void purple() {
    if (Constants.ledEnabled) {
      led1.purple();
      led2.purple();
      ledColor = LEDColor.purple;
    }
  }

  public LEDColor getLEDColor() {
    if (Constants.ledEnabled) {
      return ledColor;
    } else {
      return null;
    }
  }

  public void yellow() {
    if (Constants.ledEnabled) {
      led1.yellow();
      led2.yellow();
      ledColor = LEDColor.yellow;
    }
  }

  public void blue() {
    if (Constants.ledEnabled) {
      led1.blue();
      led2.blue();
      ledColor = LEDColor.blue;
    }
  }

  public void red() {
    if (Constants.ledEnabled) {
      led1.red();
      led2.red();
      ledColor = LEDColor.red;
    }
  }

  public void green() {
    if (Constants.ledEnabled) {
      led1.green();
      led2.green();
      ledColor = LEDColor.green;
    }
  }

  public void none() {
    if (Constants.ledEnabled) {
      led1.none();
      led2.none();
      ledColor = LEDColor.none;
    }
  }

  public enum LEDColor {
    yellow, purple, blue, red, green, none
  }

  private class LEDStrip {
    private Solenoid red;
    private Solenoid green;
    private Solenoid blue;

    private LEDStrip(int pcmID, int rPort, int gPort, int bPort) {
      red = new Solenoid(pcmID, PneumaticsModuleType.REVPH, rPort);
      green = new Solenoid(pcmID, PneumaticsModuleType.REVPH, gPort);
      blue = new Solenoid(pcmID, PneumaticsModuleType.REVPH, bPort);
    }

    public void yellow() {
      if (Constants.ledEnabled) {
        red.set(true);
        blue.set(false);
        green.set(true);
      }
    }

    public void purple() {
      if (Constants.ledEnabled) {
        red.set(true);
        green.set(false);
        blue.set(true);
      }
    }

    public void blue() {
      if (Constants.ledEnabled) {
        red.set(false);
        green.set(false);
        blue.set(true);
      }
    }

    public void red() {
      if (Constants.ledEnabled) {
        red.set(true);
        green.set(false);
        blue.set(false);
      }
    }

    public void green() {
      if (Constants.ledEnabled) {
        red.set(false);
        green.set(true);
        blue.set(false);
      }
    }

    public void none() {
      if (Constants.ledEnabled) {
        red.set(false);
        green.set(false);
        blue.set(false);
      }
    }
  }
}
