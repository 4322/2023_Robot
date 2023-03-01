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
      power1 = new Solenoid(LEDConstants.pcmID,PneumaticsModuleType.REVPH, LEDConstants.pPort1);
      power1.set(true);
      power2 = new Solenoid(LEDConstants.pcmID,PneumaticsModuleType.REVPH, LEDConstants.pPort2);
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
  public LEDColor ledColor()
  {
    return ledColor;
  }
  public void yellow() {
    if (Constants.ledEnabled) {
      led1.yellow();
      led2.yellow();
      ledColor = LEDColor.yellow;
    }
  }

  public enum LEDColor {
    yellow, purple, none
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
      red.set(true);
      green.set(true);
      blue.set(false);
    }

    public void purple() {
      red.set(true);
      green.set(false);
      blue.set(true);

    }
  }
}
