// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmRotationConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import frc.robot.extensions.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
// BYEAH \*<>*/
import edu.wpi.first.wpilibj.event.EventLoop;

public class ArmRotation extends SubsystemBase {
  CANSparkMax ArmRotation;
  XboxController xboxController;
  RelativeEncoder CurrentValue;

  DigitalInput forwardLimitSwitch;
  DigitalInput backwardLimitSwitch;

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);
    forwardLimitSwitch = new DigitalInput(0);
    backwardLimitSwitch = new DigitalInput(1);
    CurrentValue = ArmRotation.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 42);
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

 /*  ====MATH====
 Ticks per rotation, 42
  Gear Ratio Reduction, 144:1
  Gear has 30 teeth, sprocket has 12
  12 degrees per tooth? 
  144 degrees
  0.83 degrees per rotation = 42 ticks?
  .002 degrees per tick
  500 ticks for every degree */

 /* Preset Arm Positions during Auto
 (Takes in angle we need to get to) - (current encoder value) = angle we need to move
-> angle we need to move * 500 = ticks we need to move
EXAMPLE: (90 degrees) - (0 degrees) = 90 degrees ... 90 degrees * 500 = 4500 ticks
->>> Then the rotation motor is moved until it reaches 4500 ticks.
 */
 public void AutoArmRotation(double TargetAngle) {
  // GET POSITION NEEDS TESTING
double EncoderValue = CurrentValue.getPosition();
double AngleToMove = (TargetAngle - (EncoderValue/500));
double AmountToMove = AngleToMove * 500;
if(EncoderValue != AmountToMove){
ArmRotation.set(0.5);
}
else {
ArmRotation.stopMotor();
}


}

}
// 0, 1, 5, 50 , 60 (Normal arm side degrees)
// 270, 269, 265, 220, 210 (Opposite arm side degrees)
