package frc.utility;

import frc.robot.Constants;

public class OrangeMath {
  public static boolean equalToTwoDecimal(double num1, double num2) {
    double epsilon = 0.01;

    return Math.abs(num1 - num2) < epsilon;
  }

  public static double falconEncoderToMeters(double encUnits, double wheelCircumferenceMeters,
      double gearRatioWheelToMotor) {
    return (encUnits * wheelCircumferenceMeters * gearRatioWheelToMotor)
        / Constants.falconEncoderUnits;
  }

  public static double feetToMeters(double feet) {
    return feet / 3.281;
  }

  public static double metersToFeet(double meters) {
    return meters * 3.281;
  }

}
