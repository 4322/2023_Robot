package frc.utility;

import frc.robot.Constants;

public class OrangeMath {

  public static boolean equalToTwoDecimal(double num1, double num2) {
    double epsilon = 0.01;

    return Math.abs(num1 - num2) < epsilon;
  }

  public static double getCircumference(double diameter) {
    return diameter * Math.PI;
  }

  public static double falconEncoderToMeters(double encUnits, double wheelCircumferenceMeters,
      double gearRatioMotorToWheel) {
    return (encUnits * wheelCircumferenceMeters) / gearRatioMotorToWheel
        / Constants.falconEncoderUnits;
  }

  public static double feetToMeters(double feet) {
    return feet / 3.28084;
  }

  public static double metersToFeet(double meters) {
    return meters * 3.28084;
  }

  public static double inchesToMeters(double inches) {
    return inches / 39.37;
  }

  public static double metersToInches(double meters) {
    return meters * 39.37;
  }

  public static double pythag(double a, double b) {
    return Math.sqrt(a * a + b * b);  // don't use inefficient Math.pow()
  }

  // Solve for a leg
  public static double inversePythag(double hypotenuse, double leg) {
    return Math.sqrt(hypotenuse * hypotenuse - leg * leg);  // don't use inefficient Math.pow()
  }

}
