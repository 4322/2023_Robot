package frc.utility;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import frc.robot.Constants;

public class CanBusUtil {

  // stagger status frame periods to reduce peak CAN bus utilization
  private static int nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
  private static int nextShuffleboardStatusPeriodMs = Constants.shuffleboardStatusPeriodBaseMs;
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
  private static int nextSparkVerySlowStatusPeriodMs = Constants.verySlowStatusPeriodSparkBaseMs;

  public static int nextFastStatusPeriodMs() {
    if (nextFastStatusPeriodMs > Constants.fastStatusPeriodMaxMs) {
      nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
    }
    return nextFastStatusPeriodMs++;
  }

  public static int nextShuffleboardStatusPeriodMs() {
    if (nextShuffleboardStatusPeriodMs > Constants.shuffleboardStatusPeriodMaxMs) {
      nextShuffleboardStatusPeriodMs = Constants.shuffleboardStatusPeriodBaseMs;
    }
    return nextShuffleboardStatusPeriodMs++;
  }

  public static int nextSlowStatusPeriodMs() {
    if (nextSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSlowStatusPeriodMs++;
  }

  // Don't use this for Talons because they can't go this slow
  public static int nextVerySlowStatusPeriodSparkMs() {
    nextSparkVerySlowStatusPeriodMs += 11;
    return nextSparkVerySlowStatusPeriodMs;
  }

  // Stagger status frames from Talon FX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerTalonStatusFrames(WPI_TalonFX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,
        nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,
        nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }

  // Stagger status frames from Talon SRX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerTalonStatusFrames(WPI_TalonSRX talon) {
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,
        nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, nextSlowStatusPeriodMs(),
        Constants.controllerConfigTimeoutMs);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,
        nextSlowStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }

  // Stagger status frames from SPARK MAX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerSparkMax(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, nextSlowStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, nextVerySlowStatusPeriodSparkMs());
  }

  public static void fastVelocity(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
        nextFastStatusPeriodMs());
  }

  // Increase frame rates for a SPARK MAX that is being followed after initialization
  // when position control is in use.
  public static void dualSparkMaxPosCtrl(CANSparkMax mainMotor, boolean tuningMode) {
    // applied output for follower
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,
        nextFastStatusPeriodMs());
    // to detect when at target position
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2,
        nextFastStatusPeriodMs());
    if (tuningMode) {
      // to graph velocity
      mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
          nextFastStatusPeriodMs());
    }
  }

  // Increase frame rates for a SPARK MAX that is being followed after initialization
  // when velocity control is in use.
  public static void dualSparkMaxVelCtrl(CANSparkMax mainMotor, boolean tuningMode) {
    // applied output for follower
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,
        nextFastStatusPeriodMs());
    // to detect when at target speed
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
        nextFastStatusPeriodMs());
    if (tuningMode) {
      // to graph position
      mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2,
          nextFastStatusPeriodMs());
    }
  }

  // Increase frame rates for a Talon that is being followed after initialization.
  public static void dualTalonFX(WPI_TalonFX mainMotor) {
    // applied output for follower
    mainMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 
        nextFastStatusPeriodMs(), Constants.controllerConfigTimeoutMs);
  }

}
