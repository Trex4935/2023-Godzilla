// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmRotationConstants;

import com.revrobotics.CANSparkMax;
import frc.robot.extensions.*;
import edu.wpi.first.wpilibj.DigitalInput;


public class ArmRotation extends SubsystemBase {
  CANSparkMax ArmRotation;
  XboxController xboxController;


  DigitalInput forwardLimitSwitch;
  DigitalInput backwardLimitSwitch;

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);
    forwardLimitSwitch = new DigitalInput(0);
    backwardLimitSwitch = new DigitalInput(1);
       
  }
 // sets the speed that the arm moves forward
  public void moveArmForward() {
    if (forwardLimitSwitch.get()) {
      // if the forwardLimitSwitch is true, stop the motors
       ArmRotation.stopMotor();
    } else {
        // if the forwardLimitSwitch is false, then allow motor to keep moving
        ArmRotation.set(0.25);
      }
    }  

  // sets the speed that the arm moves backward
  public void moveArmBackward() {
    if ( backwardLimitSwitch.get()) {
      // if the backwardimitSwitch is true,stop the motor 
      ArmRotation.stopMotor();
    } else {
        //if the backwardLimitSwitch is false, then allow the motor to keep moving
        ArmRotation.set(0.25);
      }
    }
  

  // stops the ArmRotation motor
  public void stopArmRotation() {
    ArmRotation.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
