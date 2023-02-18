package frc.utility;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import frc.robot.Constants;

public class SparkMaxUtil {

  // stagger status frame periods to reduce peak CAN bus utilization
  private static int nextSparkSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
  private static int nextSparkVerySlowStatusPeriodMs = Constants.verySlowStatusPeriodSparkBaseMs;

  public static int nextSlowStatusPeriodSparkMs() {
    if (nextSparkSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSparkSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSparkSlowStatusPeriodMs++;
  }

  public static int nextVerySlowStatusPeriodSparkMs() {
    nextSparkVerySlowStatusPeriodMs += 11;
    return nextSparkVerySlowStatusPeriodMs;
  }

  // Stagger status frames from Talon FX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerSparkMax(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, nextSlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, nextSlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, nextSlowStatusPeriodSparkMs());
  }

}

