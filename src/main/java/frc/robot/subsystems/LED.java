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

  public enum SubstationState {
    off,  // revert to cone/cube color
    moveRight,  // solid red (toward drivers) or blue (away from drivers)
    moveRightShort,  // blinking red or blue
    moveLeft,  // solid red (toward drivers) or blue (away from drivers)
    moveLeftShort, // blinking red or blue
    aligned  // green
  }

  public enum GridState {
    off,
    moveRight,  // solid blue (move to drivers left)
    moveRightShort,  // blinking blue
    moveLeft,  // solid red (move to drivers right)
    moveLeftShort, // blinking red
    aligned  // green
  }

  public enum GamePiece {
    none,
    cone,
    cube
  }

  public enum Alignment {
    none,
    substation,
    grid
  }

  private enum LEDColor {
    yellow, purple, blue, red, green, none
  }

  private enum BlinkType {
    none, fast
  }

  LEDStrip leftLED;
  LEDStrip rightLED;
  Solenoid power1;
  Solenoid power2;
  private SubstationState lastSubstationState = SubstationState.off;
  private GridState lastGridState = GridState.off;
  private boolean intakeStalled;
  private GamePiece lastGamePiece = GamePiece.none;
  private Alignment currentAlignment = Alignment.none;
  private static LED ledSubsystem;

  public static LED getInstance() {
    if (ledSubsystem == null) {
      ledSubsystem = new LED();
    }
    return ledSubsystem;
  }

  // LED strip sides are robot relative
  private LED() {
    if (Constants.ledEnabled) {
      power1 = new Solenoid(Constants.LED.pcmID, PneumaticsModuleType.REVPH, Constants.LED.pPortLeft);
      power1.set(true);
      power2 = new Solenoid(Constants.LED.pcmID, PneumaticsModuleType.REVPH, Constants.LED.pPortRight);
      power2.set(true);
      leftLED = new LEDStrip(Constants.LED.pcmID, Constants.LED.rPortLeft, Constants.LED.gPortLeft,
          Constants.LED.bPortLeft);
      rightLED = new LEDStrip(Constants.LED.pcmID, Constants.LED.rPortRight, Constants.LED.gPortRight,
          Constants.LED.bPortRight);
      Limelight.getSubstationInstance().activateCustomAprilTag();
      Limelight.getGridInstance().activateAprilTag();
    }
  }

  @Override
  public void periodic() {
    leftLED.periodic();
    rightLED.periodic();
  }

  public void setSubstationState(SubstationState state) {
    // only update LEDs upon a change to reduce CAN bus loading
    if (lastSubstationState != state) {
      lastSubstationState = state;
      selectLED();
    }
  }

  public void setGridState(GridState state) {
    // only update LEDs upon a change to reduce CAN bus loading
    if (lastGridState != state) {
      lastGridState = state;
      selectLED();
    }
  }

  public void setIntakeStalled(boolean nowStalled) {
    // only update LEDs upon a change to reduce CAN bus loading
    if (intakeStalled != nowStalled) {
      intakeStalled = nowStalled;
      selectLED();
    }
 }

  public void setGamePiece(GamePiece gamePiece) {
    // only update LEDs upon a change to reduce CAN bus loading
    if (lastGamePiece != gamePiece) {
      lastGamePiece = gamePiece;
      selectLED();
    }
  }

  public void setAlignment(Alignment alignment) {
    // only update LEDs upon a change to reduce CAN bus loading
    if (currentAlignment != alignment) {
      currentAlignment = alignment;
      selectLED();
    }
  }

  private void selectLED() {
    if (intakeStalled && (currentAlignment == Alignment.grid) && (lastGamePiece == GamePiece.cone)) {
      Limelight.getGridInstance().activateRetroReflective();
      switch (lastGridState) {
        case off:
          leftLED.setLED(LEDColor.none, BlinkType.none);
          rightLED.setLED(LEDColor.none, BlinkType.none);
          break;
        case moveRight:
          leftLED.setLED(LEDColor.blue, BlinkType.none);
          rightLED.setLED(LEDColor.blue, BlinkType.none);
          break;
        case moveRightShort:
          leftLED.setLED(LEDColor.blue, BlinkType.fast);
          rightLED.setLED(LEDColor.blue, BlinkType.fast);
          break;
        case moveLeft:
          leftLED.setLED(LEDColor.red, BlinkType.none);
          rightLED.setLED(LEDColor.red, BlinkType.none);
          break;
        case moveLeftShort:
          leftLED.setLED(LEDColor.red, BlinkType.fast);
          rightLED.setLED(LEDColor.red, BlinkType.fast);
          break;
        case aligned:
          leftLED.setLED(LEDColor.green, BlinkType.none);
          rightLED.setLED(LEDColor.green, BlinkType.none);
          break;
      }
    } else if (!intakeStalled && (currentAlignment == Alignment.substation)) {
      Limelight.getGridInstance().activateAprilTag();  // stop the blinding light
      if (Robot.getAllianceColor() == Alliance.Blue) {
        setGamePieceColor(rightLED);
        switch (lastSubstationState) {
          case off:
            setGamePieceColor(leftLED);
            break;
          case moveRight:
            leftLED.setLED(LEDColor.blue, BlinkType.none);
            break;
          case moveRightShort:
            leftLED.setLED(LEDColor.blue, BlinkType.fast);
            break;
          case moveLeft:
            leftLED.setLED(LEDColor.red, BlinkType.none);
            break;
          case moveLeftShort:
            leftLED.setLED(LEDColor.red, BlinkType.fast);
            break;
          case aligned:
            leftLED.setLED(LEDColor.green, BlinkType.none);
            break;
        }
      } else if (Robot.getAllianceColor() == Alliance.Red) {
        setGamePieceColor(leftLED);
        switch (lastSubstationState) {
          case off:
            setGamePieceColor(rightLED);
            break;
          case moveRight:
            rightLED.setLED(LEDColor.blue, BlinkType.none);
            break;
          case moveRightShort:
            rightLED.setLED(LEDColor.blue, BlinkType.fast);
            break;
          case moveLeft:
            rightLED.setLED(LEDColor.red, BlinkType.none);
            break;
          case moveLeftShort:
            rightLED.setLED(LEDColor.red, BlinkType.fast);
            break;
          case aligned:
            rightLED.setLED(LEDColor.green, BlinkType.none);
            break;
        }
      } else {
        // unknown alliance
        setGamePieceColor(leftLED);
        setGamePieceColor(rightLED);
      }
    } else if (!intakeStalled) {
      Limelight.getGridInstance().activateAprilTag();  // stop the blinding light
      setGamePieceColor(leftLED);
      setGamePieceColor(rightLED);
    } else {
      Limelight.getGridInstance().activateAprilTag();  // stop the blinding light
      leftLED.setLED(LEDColor.none, BlinkType.none);
      rightLED.setLED(LEDColor.none, BlinkType.none);
    }
  }

  private void setGamePieceColor(LEDStrip led) {
    switch (lastGamePiece) {
      case cone:
        led.setLED(LEDColor.yellow, BlinkType.none);
        break;
      case cube:
        led.setLED(LEDColor.purple, BlinkType.none);
        break;
      case none:
        led.setLED(LEDColor.none, BlinkType.none);
        break;  
    }
  }

  private class LEDStrip {
    private Solenoid redPort;
    private Solenoid greenPort;
    private Solenoid bluePort;

    private LEDColor lastLEDColor;
    private BlinkType lastBlinkType;
    private boolean blinkOn;
    private Timer blinkTimer = new Timer();

    private LEDStrip(int pcmID, int rPort, int gPort, int bPort) {
      redPort = new Solenoid(pcmID, PneumaticsModuleType.REVPH, rPort);
      greenPort = new Solenoid(pcmID, PneumaticsModuleType.REVPH, gPort);
      bluePort = new Solenoid(pcmID, PneumaticsModuleType.REVPH, bPort);
      setLED(LEDColor.none, BlinkType.none);
    }

    private void setLED(LEDColor color, BlinkType blink) {
      if ((color != lastLEDColor) || (blink != lastBlinkType)) {
        lastLEDColor = color;
        lastBlinkType = blink;
        blinkOn = true;
        activateLED();
        blinkTimer.restart();
      }
    }

    private void periodic() {
      if ((lastBlinkType == BlinkType.fast) && blinkTimer.hasElapsed(Constants.LED.blinkFastSec)) {
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
          break;
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
