// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmRotationConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.extensions.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
// BYEAH \*<>*/

public class ArmRotation extends SubsystemBase {
  CANSparkMax ArmRotation;
  XboxController xboxController;
  RelativeEncoder armEncoder;

  DigitalInput forwardLimitSwitch;
  DigitalInput backwardLimitSwitch;

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);
    forwardLimitSwitch = new DigitalInput(0);
    backwardLimitSwitch = new DigitalInput(1);
    armEncoder = ArmRotation.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 42);
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

  // 1.Neo x
  // 2.Gear Ration(math)x
  // 3.Limit Switches x
  // 4.Encoder

 //Ticks per rotation, 42
 //Gear Ratio Reduction, 144:1
 //Gear has 30 teeth, sprocket has 12
 //  12 degrees per tooth? 
 // 144 degrees
 // 0.83 degrees per rotation = 42 ticks?
 // .002 degrees per tick
 // 500 ticks for every degree
 public void AutoArmRotation(int angle) {
 int AmountTurn = angle * 500;
 //ArmRotation.moveArmForward(AmountTurn);
 }

}
