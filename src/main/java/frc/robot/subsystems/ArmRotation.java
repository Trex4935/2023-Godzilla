// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.robot.extensions.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

public class ArmRotation extends SubsystemBase {
  CANSparkMax armRotationMotor;
  XboxController xboxController;
  RelativeEncoder armEncoder;

  DigitalInput forwardLimitSwitch;
  DigitalInput backwardLimitSwitch;

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    armRotationMotor = SparkMax.createDefaultCANSparkMax(Constants.armRotationCAN);

    forwardLimitSwitch = new DigitalInput(0);
    backwardLimitSwitch = new DigitalInput(1);
    armEncoder = armRotationMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 42);
  }

  /** Sets the speed that the arm moves forward */
  public void moveArmForward() {
    if (forwardLimitSwitch.get()) {
      // if the forwardLimitSwitch is true, stop the motors
      armRotationMotor.stopMotor();
    } else {
      // if the forwardLimitSwitch is false, then allow motor to keep moving
      armRotationMotor.set(0.25);
    }
  }

  /** sets the speed that the arm moves backward */
  public void moveArmBackward() {
    if (backwardLimitSwitch.get()) {
      // if the backwardimitSwitch is true,stop the motor
      armRotationMotor.stopMotor();
    } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      armRotationMotor.set(0.25);
    }
  }

  /** stops the ArmRotation motor */
  public void stopArmRotation() {
    armRotationMotor.stopMotor();
  }

  /*
   * ====MATH====
   * Ticks per rotation, 42
   * Gear Ratio Reduction, 144:1
   * Gear has 30 teeth, sprocket has 12
   * 12 degrees per tooth?
   * 144 degrees
   * 0.83 degrees per rotation = 42 ticks?
   * .002 degrees per tick
   * 500 ticks for every degree
   */

  /*
   * Preset Arm Positions during Auto
   * (Takes in angle we need to get to) - (current encoder value) = angle we need
   * to move
   * -> angle we need to move * 500 = ticks we need to move
   * EXAMPLE: (90 degrees) - (0 degrees) = 90 degrees ... 90 degrees * 500 = 4500
   * ticks
   * ->>> Then the rotation motor is moved until it reaches 4500 ticks.
   */

  public boolean AutoArmRotation(double TargetAngle) {
    double encoderValueTicks = armEncoder.getPosition();
    double targetAngleTicks = TargetAngle * 500;
    double changesign = Math.signum(targetAngleTicks - encoderValueTicks);
    // determine direction of arm movement based on sign of encoder differences
    if (changesign > 0) {
      moveArmForward();
      // return boolean based on if current encoder value is within +- 100 ticks of target
      return Helper.RangeCompare(targetAngleTicks + 100, targetAngleTicks - 100, encoderValueTicks);
    } else {
      moveArmBackward();
      // return boolean based on if current encoder value is within +- 100 ticks of target
      return Helper.RangeCompare(targetAngleTicks + 100, targetAngleTicks - 100, encoderValueTicks);
    }
  }

        // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", null, null);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
// 0, 1, 5, 50 , 60 (Normal arm side degrees)
// 270, 269, 265, 220, 210 (Opposite arm side degrees)
