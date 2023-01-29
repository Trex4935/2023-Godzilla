// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;

import frc.robot.extensions.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

public class ArmRotation extends SubsystemBase {
  CANSparkMax armRotationMotor;
  SparkMaxPIDController armRotationPID;
  XboxController xboxController;
  RelativeEncoder armEncoder;

  DigitalInput forwardLimitSwitch;
  DigitalInput backwardLimitSwitch;

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    armRotationMotor = SparkMax.createDefaultCANSparkMax(Constants.armRotationCAN);
    armRotationMotor = SparkMax.configPIDwithSmartMotion(armRotationMotor, 0.03, 0, 0, 0, 0, 10000, 400, 0);

    forwardLimitSwitch = new DigitalInput(0);
    backwardLimitSwitch = new DigitalInput(1);
    armEncoder = armRotationMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 42);
  }

  /** determines if the arm is in the red zone or not, and if it is extended or not */
  public boolean armRedZone() {
    // if arm is in red zone and it is extended
    if (Helper.RangeCompare(90000, 45000, armEncoder.getPosition()) && (Constants.isRetracted == false)) {
      Constants.inRedZone = true; // Updates global variable
      return true;
    } else {
      Constants.inRedZone = false; // Updates global variable
      return false;
    }
  }

  /** Sets the speed that the arm moves forward */
  public void moveArmForward() {
    // if either fwrd limit switch or it is in red zone and extended, stop motor
    if (forwardLimitSwitch.get() || (armRedZone())) {
      armRotationMotor.stopMotor();
    } else {
      // if the forwardLimitSwitch is false, then allow motor to keep moving
      armRotationMotor.set(Constants.armRotateSpeed);
    }
  }

  /** sets the speed that the arm moves backward */
  public void moveArmBackward() {
    // if either bckwrd limit switch or it is in red zone and extended, stop motor
    if (backwardLimitSwitch.get() || (armRedZone())) {
      armRotationMotor.stopMotor();
    } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      armRotationMotor.set(Constants.armRotateSpeed * (-1));
    }
  }

 /** Returns the encoder value */
  public double getEncoderValue() {
    return armEncoder.getPosition();
  }

  /** Returns the angle */
  public double getArmAngle() {
    return getEncoderValue() / 500;
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

  /** Rotates the arm */
  public void AutoArmRotation(double TargetAngle) {
    double encoderValueTicks = armEncoder.getPosition(); // Gets ticks
    double targetAngleTicks = TargetAngle * 500; // Converts target angle to ticks.
    double checkSign = Math.signum(targetAngleTicks - encoderValueTicks); // Determines the sign of the direction
    // determine direction of arm movement based on sign of encoder differences
    if (Helper.RangeCompare(targetAngleTicks + 100, targetAngleTicks - 100, encoderValueTicks) == false) { // If not in range then move...
      if (checkSign > 0) { // If sign is positive rotate compressor-side.
        moveArmForward();
      } else { // If sign not positive rotate battery-side.
        moveArmBackward();
      }
    } else { // If in range then stop motor.
      stopArmRotation();
    }
  }

  // Sendable override
  // Anything put here will be added to the network tables and thus can be added
  // to the dashboard / consumed by the LED controller
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Angle", this::getArmAngle, this::AutoArmRotation);
      builder.addDoubleProperty("Encoder Value", this::getEncoderValue, null);
      builder.addBooleanProperty("RedZone",this::armRedZone, null);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
// 0, 1, 5, 50 , 60 (Normal arm side degrees)
// 270, 269, 265, 220, 210 (Opposite arm side degrees)
