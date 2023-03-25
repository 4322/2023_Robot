package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Offers basic control of RGB (three-color) LEDs, given a PCM ID as well as RGB ports
 */
public class LED extends SubsystemBase {

  // right/left enum directions are relative to the limelight view

  public enum substationState {
    off,  // revert to cone/cube color
    moveRight,  // solid red (toward drivers) or blue (away from drivers)
    moveRightShort,  // blinking red or blue
    moveLeft,  // solid red (toward drivers) or blue (away from drivers)
    moveLeftShort, // blinking red or blue
    aligned  // green
  }

  public enum gridState {
    off,
    moveRight,  // solid blue (move to drivers left)
    moveRightShort,  // blinking blue
    moveLeft,  // solid red (move to drivers right)
    moveLeftShort, // blinking red
    aligned  // green
  }

  public enum loadState {
    none,
    cone,
    cube
  }

  public enum alignment {
    none,
    substation,
    grid
  }

  private enum LEDColor {
    yellow, purple, blue, red, green, none
  }

  private enum blinkType {
    none, fast
  }

  LEDStrip leftLED;
  LEDStrip rightLED;
  Solenoid power1;
  Solenoid power2;
  private substationState lastSubstationState;
  private gridState lastGridState;
  private boolean intakeStalled;
  private loadState lastLoadState;
  private alignment alignmentState;

  // LED strip sides are robot relative
  public LED() {
    if (Constants.ledEnabled) {
      power1 = new Solenoid(Constants.LED.pcmID, PneumaticsModuleType.REVPH, Constants.LED.pPortLeft);
      power1.set(true);
      power2 = new Solenoid(Constants.LED.pcmID, PneumaticsModuleType.REVPH, Constants.LED.pPortRight);
      power2.set(true);
      leftLED = new LEDStrip(Constants.LED.pcmID, Constants.LED.rPortLeft, Constants.LED.gPortLeft,
          Constants.LED.bPortLeft);
      rightLED = new LEDStrip(Constants.LED.pcmID, Constants.LED.rPortRight, Constants.LED.gPortRight,
          Constants.LED.bPortRight);
    }
  }

  @Override
  public void periodic() {
    leftLED.periodic();
    rightLED.periodic();
  }

  public void setSubstationState(substationState state) {
    lastSubstationState = state;
    selectLED();
  }

  public void setGridState(gridState state) {
    lastGridState = state;
    selectLED();
  }

  public void setIntakeStalled(boolean stalled) {
    intakeStalled = stalled;
    selectLED();
  }

  public void setLoadState(loadState state) {
    lastLoadState = state;
    selectLED();
  }

  private void selectLED() {
    if (intakeStalled && (alignmentState == alignment.grid) && (lastLoadState == loadState.cone)) {
      switch (lastGridState) {
        case off:
          leftLED.setLED(LEDColor.none, blinkType.none);
          rightLED.setLED(LEDColor.none, blinkType.none);
          break;
        case moveRight:
          leftLED.setLED(LEDColor.blue, blinkType.none);
          rightLED.setLED(LEDColor.blue, blinkType.none);
          break;
        case moveRightShort:
          leftLED.setLED(LEDColor.blue, blinkType.fast);
          rightLED.setLED(LEDColor.blue, blinkType.fast);
          break;
        case moveLeft:
          leftLED.setLED(LEDColor.red, blinkType.none);
          rightLED.setLED(LEDColor.red, blinkType.none);
          break;
        case moveLeftShort:
          leftLED.setLED(LEDColor.red, blinkType.fast);
          rightLED.setLED(LEDColor.red, blinkType.fast);
          break;
        case aligned:
          leftLED.setLED(LEDColor.green, blinkType.none);
          rightLED.setLED(LEDColor.green, blinkType.none);
          break;
      }
    } else if (!intakeStalled && (alignmentState == alignment.substation)) {
      if (Robot.getAllianceColor() == Alliance.Blue) {

        switch (lastSubstationState) {
          case off:
            break;
          case moveRight:
            leftLED.setLED(LEDColor.blue, blinkType.none);
            break;
          case moveRightShort:
            leftLED.setLED(LEDColor.blue, blinkType.fast);
            break;
          case moveLeft:
            leftLED.setLED(LEDColor.red, blinkType.none);
            break;
          case moveLeftShort:
            leftLED.setLED(LEDColor.red, blinkType.fast);
            break;
          case aligned:
            leftLED.setLED(LEDColor.green, blinkType.none);
            break;
        }
      }
    } else {
      leftLED.setLED(LEDColor.none, blinkType.none);
      rightLED.setLED(LEDColor.none, blinkType.none);
    }
  }

  private class LEDStrip {
    private Solenoid redPort;
    private Solenoid greenPort;
    private Solenoid bluePort;

    private LEDColor lastLEDColor;
    private blinkType lastBlinkType;
    private boolean blinkOn;
    private Timer blinkTimer = new Timer();

    private LEDStrip(int pcmID, int rPort, int gPort, int bPort) {
      redPort = new Solenoid(pcmID, PneumaticsModuleType.REVPH, rPort);
      greenPort = new Solenoid(pcmID, PneumaticsModuleType.REVPH, gPort);
      bluePort = new Solenoid(pcmID, PneumaticsModuleType.REVPH, bPort);
      setLED(LEDColor.none, blinkType.none);
    }

    private void setLED(LEDColor color, blinkType blink) {
      lastLEDColor = color;
      lastBlinkType = blink;
      blinkOn = true;
      activateLED();
      blinkTimer.restart();
    }

    private void periodic() {
      if ((lastBlinkType == blinkType.fast) && blinkTimer.hasElapsed(Constants.LED.blinkFastSec)) {
        if (blinkOn) {
          deactivateLED();
          blinkOn = false;
        } else {
          activateLED();
          blinkOn = true;
        }
        blinkTimer.restart();
      }
    }

    private void deactivateLED() {
      redPort.set(false);
      greenPort.set(false);
      bluePort.set(false);
    }

    private void activateLED() {
      switch (lastLEDColor) {
        case none:
          deactivateLED();
        case yellow:
          redPort.set(true);
          bluePort.set(false);
          greenPort.set(true);
          break;
        case purple:
          redPort.set(true);
          greenPort.set(false);
          bluePort.set(true);
          break;
        case blue:
          redPort.set(false);
          greenPort.set(false);
          bluePort.set(true);
          break;
        case red:
          redPort.set(true);
          greenPort.set(false);
          bluePort.set(false);
          break;
        case green:
          redPort.set(false);
          greenPort.set(true);
          bluePort.set(false);
          break;
      }
    }
  }
}
