package frc.utility;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class TalonUtil {

  // stagger status frame periods to reduce peak CAN bus utilization
  private static int nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;

  public static int nextFastStatusPeriodMs() {
    if (nextFastStatusPeriodMs > Constants.fastStatusPeriodMaxMs) {
      nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
    }
    return nextFastStatusPeriodMs++;
  }

  public static int nextSlowStatusPeriodMs() {
    if (nextSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSlowStatusPeriodMs++;
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

}
