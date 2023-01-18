// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmRotationConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.extensions.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;


public class ArmRotation extends SubsystemBase {
  CANSparkMax ArmRotation;
  XboxController xboxController;

  //init a DigitalInput on channels 0 and 1; top is forward and bottom is backward
  DigitalInput forwardlimitSwitch = new DigitalInput(0);
  DigitalInput backwardlimitSwitch = new DigitalInput(1);

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);
       // sets the speed that the arm moves forward (now declares the limit switch)
  }

  public void moveArmForward() {
    if (forwardlimitSwitch.get()) {
      // if the toplimitSwitch is true, stop the motors
       ArmRotation.stopMotor();
    } else {
        // if the toplimitSwitch is false, then allow motor to keep moving
        ArmRotation.set(0.25);
      }
    }  

  // sets the speed that the arm moves backward (now declares the limit switch)
  public void moveArmBackward() {
    if ( backwardlimitSwitch.get()) {
      // if the bottomlimitSwitch is true,stop the motor 
      ArmRotation.stopMotor();
    } else {
        //if the bottomlimitSwitch is false, then allow the motor to keep moving
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
