package frc.robot.subsystems;

import java.lang.annotation.Target;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmDirection;

public class Arm extends SubsystemBase {
  private CANSparkMax motor;
  private Double currentTarget = null;
  private ArmDirection armDirection;

  public Arm() {
    if (Constants.armEnabled) {
      motor = new CANSparkMax(Constants.ArmConstants.motorID, MotorType.kBrushless);
    }
  }

  public void init() {
    if (Constants.armEnabled) {
      motor.restoreFactoryDefaults();
      motor.setIdleMode(IdleMode.kCoast);
      motor.setOpenLoopRampRate(Constants.ArmConstants.rampRate);
    }
  }

  public double getPosition() {
    if (Constants.armEnabled) {
      return motor.getEncoder().getPosition();
    } else {
      return -1;
    }
  }

  public boolean isAtTarget() {
    if (!Constants.armEnabled) {
      return true;
    }
    return (Math.abs(getPosition() - currentTarget) <= ArmConstants.positionTolerance);
  }

  public boolean rotateToPosition(double targetPosition) {
    if (Constants.armEnabled) {
      if ((targetPosition > ArmConstants.minPosition)
          && (targetPosition < ArmConstants.maxPosition)) {
        motor.getPIDController().setReference(targetPosition, ControlType.kPosition);
        currentTarget = targetPosition;
        DataLogManager
            .log("Rotating to position " + currentTarget + " from position " + getPosition());
        return true;
      }
    }
    return false;
  }

  public void rotateForward() {
    motor.set(ArmConstants.forward);
    DataLogManager.log("Arm rotating forward");
  }

  public void rotateBackward() {
    motor.set(ArmConstants.backward);
    DataLogManager.log("Arm rotating backward");
  }

  public void setCoastMode() {
    if (Constants.armEnabled) {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.armEnabled) {
      motor.setIdleMode(IdleMode.kBrake);
    }
  }

  @Override
  public void periodic() {
    if (Constants.armEnabled) {
      if (logTimer.hasElapsed(ArmConstants.logIntervalSeconds)) {
        DataLogManager.log("Arm position: " + getPosition());
        logTimer.reset();
      }
    }
  }
}
