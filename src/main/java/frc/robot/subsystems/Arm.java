// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.Falcon;
import frc.robot.extensions.Helper;
import frc.robot.extensions.SparkMax;

public class Arm extends SubsystemBase {

 // Arm Extension 
 private WPI_TalonFX ArmExtensionMotor;

 DigitalInput armRetractedLimitSwitch;

 // Arm Rotation 
 CANSparkMax armRotationMotor;
 RelativeEncoder armRotationEncoder;

 DigitalInput compressorSideLimitSwitch;
 DigitalInput batterySideLimitSwitch;


  /** Creates a new Arm. */
  public Arm() {
    // init motors
    ArmExtensionMotor = Falcon.createDefaultFalcon(Constants.armExtensionCAN);
    armRotationMotor = SparkMax.createDefaultCANSparkMax(Constants.armRotationCAN);

    // Arm Extension Limit Switches
    armRetractedLimitSwitch = new DigitalInput(3);
    // Arm Rotation Limit Switches
    compressorSideLimitSwitch = new DigitalInput(0);
    batterySideLimitSwitch = new DigitalInput(1);

   // rotation encoder init
   armRotationEncoder = armRotationMotor.getEncoder(); 
  }

   // Arm Extension Methods

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
      ArmExtensionMotor.set(Constants.armExtensionSpeed);
  }

  /** Sets the speed that the arm moves backward */
  public void retractArm() {
    if (armRetractedLimitSwitch.get()) {
      // if the backwardimitSwitch is true,stop the motor
      ArmExtensionMotor.stopMotor();
    } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      ArmExtensionMotor.set(Constants.armExtensionSpeed);
    }
  }
  // __________________________
// rename later as extend and retract 
public void moveArmLeft() {
  ArmExtensionMotor.set(Constants.armExtensionSpeed);
  DataLogManager.log("MOVING LEFT");
}

public void moveArmRight() {
  ArmExtensionMotor.set((-1) * Constants.armExtensionSpeed);
  DataLogManager.log("MOVING RIGHT");
}

// __________________________

// method that determines if the arm is retracted or not
  public boolean fullyRetracted() {
    if (armRetractedLimitSwitch.get()) {
      // updating global
      Constants.isRetracted = true;
      return true;
    } else {
      Constants.isRetracted = false;
      return false;
    }
  } 

  /** Extends or retracts the the arm */
  public void AutoArmExtension(double TargetDistance) { // Distance Unit is: ?????
    double encoderValueTicks = ArmExtensionMotor.getSelectedSensorPosition(); // Gets ticks
    double targetDistanceTicks = TargetDistance * Constants.inchPerExtentionTicks; // Converts target distance to ticks.
    double checkSign = Math.signum(targetDistanceTicks - encoderValueTicks); // Determines the sign of the direction
    // determine direction of arm movement based on sign of encoder differences
    if (Helper.RangeCompare(targetDistanceTicks + 100, targetDistanceTicks - 100, encoderValueTicks) == false) { // If not in range then move...
      if (checkSign > 0) { // If sign is positive move forward.
        extendArm();
      } else { // If sign not positive move backward.
        retractArm();
      }
    } else { // If in range then stop motor.
      stopExtensionMotor();
    }
  }

  // Sendable Methods
  /** Gets Encoder Ticks for Extension Encoder (Sendable) */
  public double getExtensionEncoderTicks() {
    return ArmExtensionMotor.getSelectedSensorPosition();
  }

  /** Gets Speed of Arm Extension Motor (Sendable) */
  public double getExtensionMotorSpeed() {
    return ArmExtensionMotor.get();
  }

  public String getExtensionPosition() {

    
    return "Error";
  }

   // Arm Rotation Methods
/** determines if the arm is in the red zone or not, and if it is extended or not */
  public boolean armRedZone() {
    // if arm is in red zone and it is extended
    if (Helper.RangeCompare(90000, 45000, armRotationEncoder.getPosition()) && (Constants.isRetracted == false)) {
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
    // if (compressorSideLimitSwitch.get() || (armRedZone())) {
    //   armRotationMotor.stopMotor();
    // } else {
      // if the forwardLimitSwitch is false, then allow motor to keep moving
      armRotationMotor.set(Constants.armRotateSpeed);
    // }
  }

  /** sets the speed that the arm moves backward */
  public void moveArmBackward() {
    // if either bckwrd limit switch or it is in red zone and extended, stop motor
    // if (batterySideLimitSwitch.get() || (armRedZone())) {
    //  armRotationMotor.stopMotor();
    // } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      armRotationMotor.set(Constants.armRotateSpeed * (-1));
    // }
  }

// __________________________

public void moveArmCompressor() {
  armRotationMotor.set(Constants.armRotateSpeed);
  DataLogManager.log("MOVING COMP");
}

public void moveArmBattery() {
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
   *  fixed math 2/4/23
   *  small gear has 15 (40/15 ratio = 2.67), big circle (gear?) has 40... apparently its 9 degrees per tooth now?
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
    if (!Helper.RangeCompare(targetAngleTicks + 10, targetAngleTicks - 10, encoderValueTicks)) { // If not in range then move...
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
      
// Arm Extension Sendables
    builder.addDoubleProperty("Extension", null, null);
    builder.addDoubleProperty("Extension Encoder Position", this::getExtensionEncoderTicks, null);
    builder.addDoubleProperty("Extension Motor Rotation", this::getExtensionMotorSpeed, null);
    builder.addStringProperty("Arm Extension Position", this::getExtensionPosition, null);
    builder.addBooleanProperty("Is Retracted", this::fullyRetracted, null);

// Arm Rotation Sendables
      builder.addDoubleProperty("Angle", this::getArmAngle, this::AutoArmRotation);
      builder.addDoubleProperty("Rotation Encoder", this::getEncoderValue, null);
      builder.addBooleanProperty("RedZone",this::armRedZone, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
         // Arm Rotation Notes
// 0, 1, 5, 50 , 60 (Normal arm side degrees)
// 270, 269, 265, 220, 210 (Opposite arm side degrees)
