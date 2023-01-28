// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.extensions.*;

public class ArmExtension extends SubsystemBase {
  // Motor
  private WPI_TalonFX ArmExtensionMotor;

  // Limit switches
  DigitalInput outerLimitSwitch;
  DigitalInput innerLimitSwitch;

  /** Creates a new motor. */
  public ArmExtension() {
    // Arm Extension
    ArmExtensionMotor = Falcon.createDefaultFalcon(Constants.armExtensionCAN);
    /// Need Encoder based soft limits implemented

    outerLimitSwitch = new DigitalInput(2);
    innerLimitSwitch = new DigitalInput(3);

  }

  /** Stops the extension motor */
  public void stopExtensionMotor() {
    ArmExtensionMotor.stopMotor();
  }

  /**
   * Takes a speed values from -1 to 1 and set the extension motor to that value
   */
  public void extendArmSetSpeed(Double speed) {
    ArmExtensionMotor.set(speed);
  }

  /** Sets the speed that the arm moves outward */
  public void extendArm() {
    if (outerLimitSwitch.get()) {
      // if the forwardLimitSwitch is true, stop the motors
      ArmExtensionMotor.stopMotor();
    } else {
      // if the forwardLimitSwitch is false, then allow motor to keep moving
      ArmExtensionMotor.set(0.25);
    }
  }

  /** Sets the speed that the arm moves backward */
  public void retractArm() {
    if (innerLimitSwitch.get()) {
      // if the backwardimitSwitch is true,stop the motor
      ArmExtensionMotor.stopMotor();
    } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      ArmExtensionMotor.set(0.25);
    }
  }
// method that determines if the arm is retracted or not
  public boolean fullyRetracted() {
    if (innerLimitSwitch.get()) {
      // updating global
      Constants.isRetracted = true;
      return true;
    } else {
      Constants.isRetracted = false;
      return false;
    }
  } 

  /** Extends or retracts the the arm */
  // Distance Unit is: ?????
  public boolean AutoArmExtension(double TargetDistance) {
    double encoderValueTicks = ArmExtensionMotor.getSelectedSensorPosition();
    double targetDistanceTicks = TargetDistance * 2048;
    double changesign = Math.signum(targetDistanceTicks - encoderValueTicks);
    // determine direction of arm movement based on sign of encoder differences
    if (changesign > 0) {
      extendArm();
      // return boolean based on if current encoder value is within +- 100 ticks (change later?) of target
      return Helper.RangeCompare(targetDistanceTicks + 100, targetDistanceTicks - 100, encoderValueTicks);
    } else {
      retractArm();
      // return boolean based on if current encoder value is within +- 100 ticks (change later?) of target
      return Helper.RangeCompare(targetDistanceTicks + 100, targetDistanceTicks - 100, encoderValueTicks);
    }
  }

  // Sendable override
  /*
   * Anything put here will be added to the network tables and thus can be added
   * to the dashboard / consumed by the LED controller
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Extension", null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
