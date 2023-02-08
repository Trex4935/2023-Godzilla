// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.Falcon;
import frc.robot.extensions.FlippedDIO;
import frc.robot.extensions.Helper;
import frc.robot.extensions.SparkMax;

public class Arm extends SubsystemBase {

  // Arm Extension
  private WPI_TalonFX armExtensionMotor;

  DigitalInput armRetractedLimitSwitch;

  // Arm Rotation
  CANSparkMax armRotationMotor;
  RelativeEncoder armRotationEncoder;

  DigitalInput compressorSideLimitSwitch;
  DigitalInput batterySideLimitSwitch;

  private WPI_TalonFX _talon;

  /** Creates a new Arm. */
  public Arm() {
    // init motors
    armExtensionMotor = Falcon.createDefaultFalcon(Constants.armExtensionCAN);
    armRotationMotor = SparkMax.createDefaultCANSparkMax(Constants.armRotationCAN);

    // Arm Extension Limit Switches
    armRetractedLimitSwitch = new FlippedDIO(0);
    // Arm Rotation Limit Switches
    compressorSideLimitSwitch = new DigitalInput(5);
    batterySideLimitSwitch = new DigitalInput(6);

    // rotation encoder init
    armRotationEncoder = armRotationMotor.getEncoder();
    Falcon.configMotionMagic(armExtensionMotor, 0.001,0,0,0.1,500,500);

  }

  // Arm Extension Methods

  /** Stops the extension motor */
  public void stopExtensionMotor() {
    armExtensionMotor.stopMotor();
  }

  /**
   * Takes a speed values from -1 to 1 and set the extension motor to that value
   */
  public void extendArmSetSpeed(Double speed) {
    armExtensionMotor.set(speed);
  }

  /** Sets the speed that the arm moves outward */
  public void extendArm() {
    armExtensionMotor.set(-Constants.armExtensionSpeed);
  }

  // Using motion magic set the arm to a given position
  public void setArmPostionMM(double armPositionTicks){
    armExtensionMotor.set(TalonFXControlMode.MotionMagic, armPositionTicks);
  }

  /** Sets the speed that the arm moves backward */
  public void retractArm() {
    if (armRetractedLimitSwitch.get()) {
      // if the backwardimitSwitch is true,stop the motor
      armExtensionMotor.stopMotor();
      zeroEncoder();
    } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      armExtensionMotor.set(Constants.armExtensionSpeed);
    }
  }

  // __________________________
  // rename later as extend and retract
  public void moveArmLeft() {
    armExtensionMotor.set(Constants.armExtensionSpeed);
    DataLogManager.log("MOVING LEFT");
  }

  public void moveArmRight() {
    armExtensionMotor.set((-1) * Constants.armExtensionSpeed);
    DataLogManager.log("MOVING RIGHT");
  }

  // __________________________

  // method that determines if the arm is retracted or not

  /** Extends or retracts the the arm */
  public void AutoArmExtension(double TargetDistance) { // Distance Unit is: ?????
    double encoderValueTicks = armExtensionMotor.getSelectedSensorPosition(); // Gets ticks
    double targetDistanceTicks = TargetDistance * Constants.inchPerExtentionTicks; // Converts target distance to ticks.
    double checkSign = Math.signum(targetDistanceTicks - encoderValueTicks); // Determines the sign of the direction
    // determine direction of arm movement based on sign of encoder differences
    if (!Helper.RangeCompare(targetDistanceTicks + 200, targetDistanceTicks - 200, encoderValueTicks)) {
      if (checkSign > 0) { // If sign is positive move forward.
        retractArm();
      } else { // If sign not positive move backward.
        extendArm();
      }
    } else { // If in range then stop motor.
      stopExtensionMotor();
    }
  }

  public void AutoArmExtensionMM(double targetTicks){
    // armExtensionMotor.setSelectedSensorPosition(targetTicks, 0, 20);
  }

  // Sendable Methods
  /** Gets Encoder Ticks for Extension Encoder (Sendable) */
  public double getExtensionEncoderTicks() {
    return armExtensionMotor.getSelectedSensorPosition();
  }

  /** Gets Speed of Arm Extension Motor (Sendable) */
  public double getExtensionMotorSpeed() {
    return armExtensionMotor.get();
  }

  public String getExtensionPosition() {

    return "Error";
  }

  public void zeroEncoder() {
    armExtensionMotor.setSelectedSensorPosition(0, 0, 20);
  }
  
  // Arm Rotation Methods
  /**
   * determines if the arm is in the red zone or not, and if it is extended or not
   */
  public boolean armRedZone() {
    // if arm is in red zone and it is extended
    if (Helper.RangeCompare(225, 91, armRotationEncoder.getPosition())) {
      Constants.inRedZone = true; // Updates global variable
      return true;
    } else {
      Constants.inRedZone = false; // Updates global variable
      return false;
    }
  }

  /** Sets the speed that the arm moves forward */
  public void moveArmCompressor() {
  
    // Check if we are in the red
    if (armRedZone()){
      // If arm is retracted then we can move
      if (getArmRetractedLimitSwitch()){
        armRotationMotor.set(-Constants.armRotateSpeed);
      }
      // if not retracted then stop moving
      else {
        armRotationMotor.stopMotor();
      }
    }
    // For when we are not in red zone
    else {
      // If we are not in the red zone then we just need to stop if the limit is hit
      if (!getCompressorLimitSwitch()){
        armRotationMotor.stopMotor();
      }
      // else we can move
      else {
        armRotationMotor.set(-Constants.armRotateSpeed);
      }
    }
  }

  /** sets the speed that the arm moves battery-side */
  public void moveArmBattery() {
    // Check if we are in the red
    if (armRedZone()){
      // If arm is retracted then we can move
      if (getArmRetractedLimitSwitch()){
        armRotationMotor.set(Constants.armRotateSpeed);
      }
      // if not retracted then stop moving
      else {
        armRotationMotor.stopMotor();
      }
    }
    // For when we are not in red zone
    else {
      // If we are not in the red zone then we just need to stop if the limit is hit
      if (getBatteryLimitSwitch()){
        armRotationMotor.stopMotor();
      }
      // else we can move
      else {
        armRotationMotor.set(Constants.armRotateSpeed);
      }
    }
  }

  // __________________________

  public void manualMoveArmCompressor() {
    armRotationMotor.set(Constants.armRotateSpeed);
    DataLogManager.log("MOVING COMP");
  }

  public void manualMoveArmBattery() {
    armRotationMotor.set((-1) * Constants.armRotateSpeed);
    DataLogManager.log("MOVING BATT");
  }

  // __________________________

  /** Returns the encoder value */
  public double getEncoderValue() {
    return armRotationEncoder.getPosition();
  }

  /** Returns the angle */
  public double getArmAngle() {
    return getEncoderValue() / 500;
  }

  /** stops the ArmRotation motor */
  public void stopArmRotation() {
    armRotationMotor.stopMotor();
  }

  /** Sets the default angle value (sendable) */
  public void setDefaultAngle(double m_defaultAngle) {
    Constants.ArmCarryAngleCompressor = m_defaultAngle;
  }

  /** Get the default angle value (sendable) */
  public double getDefaultAngle() {
    return Constants.ArmCarryAngleCompressor;
  }

  public boolean getArmRetractedLimitSwitch() {
    return armRetractedLimitSwitch.get();
  }

  public boolean getCompressorLimitSwitch() {
    return compressorSideLimitSwitch.get();
  }
  
  public boolean getBatteryLimitSwitch() {
    return batterySideLimitSwitch.get();
  }

  public void setArmLength(Double m_tempArmDistance) {
    Constants.tempArmDistance = m_tempArmDistance;
  }

  public double getArmLength(){
    return Constants.tempArmDistance;
  }
  /*
   * ====MATH====
   * Ticks per rotation, 42
   * Gear Ratio Reduction, 144:1
   * Gear has 40 teeth, sprocket has 12
   * 12 degrees per tooth
   * 144 degrees-?
   * 0.83 degrees per rotation = 42 ticks?
   * .002 degrees per tick-?
   * 
   * fixed math 2/4/23
   * small gear has 15 (40/15 ratio = 2.67), big circle (gear?) has 40...
   * apparently its 9 degrees per tooth now?
   * every 6048 encoder ticks, small gear turns 2.67 times, big gear moves once
   * every 44.86 ticks, we move one degree
   * 144 x 42 = 6048
   * 6048 x 2.67 = 16148.16
   * 16148.16 / 360 = 44.856 rounded 44.86
   * 
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
    double encoderValueTicks = armRotationEncoder.getPosition(); // Gets ticks
    double targetAngleTicks = TargetAngle * Constants.degreesPerRotationTicks; // Converts target angle to ticks.
    double checkSign = Math.signum(targetAngleTicks - encoderValueTicks); // Determines the sign of the direction
    // determine direction of arm movement based on sign of encoder differences
    if (!Helper.RangeCompare(targetAngleTicks + 2, targetAngleTicks - 2, encoderValueTicks)) { // If not in range then
                                                                                               // move...
      if (checkSign < 0) { // If sign is positive rotate compressor-side.
        moveArmCompressor();
      } else { // If sign not positive rotate battery-side.
        moveArmBattery();
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

    // Arm Extension Sendables
    builder.addDoubleProperty("Extension", null, null);
    builder.addDoubleProperty("Extension Encoder Position", this::getExtensionEncoderTicks, null);
    builder.addDoubleProperty("Extension Motor Rotation", this::getExtensionMotorSpeed, null);
    builder.addStringProperty("Arm Extension Position", this::getExtensionPosition, null);
    builder.addBooleanProperty("Is Retracted", this::getArmRetractedLimitSwitch, null);

    builder.addDoubleProperty("Arm Length", this::getArmLength, this::setArmLength);

    // Arm Rotation Sendables
    builder.addDoubleProperty("Angle", this::getArmAngle, this::AutoArmRotation);
    builder.addDoubleProperty("Rotation Encoder", this::getEncoderValue, null);
    builder.addBooleanProperty("RedZone", this::armRedZone, null);
    builder.addDoubleProperty("Default Angle", this::getDefaultAngle, this::setDefaultAngle);

    builder.addBooleanProperty("Comp LS", this::getCompressorLimitSwitch, null);
    builder.addBooleanProperty("Batt LS", this::getBatteryLimitSwitch, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
// Arm Rotation Notes
// 0, 1, 5, 50 , 60 (Normal arm side degrees)
// 270, 269, 265, 220, 210 (Opposite arm side degrees)
