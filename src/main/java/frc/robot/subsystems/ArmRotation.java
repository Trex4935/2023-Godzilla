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
  //init a DigitalInput on channels 0 and 1; top is forward and bottom is backward
  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);
   //init a DigitalInput on channels 0 and 1; top is forwards and bottom is backwards
  

    // sets the speed that the arm moves forward ()
  }

  public void moveArmForward() {
    ArmRotation.set(1);
  }

  // sets the speed that the arm moves backward
  public void moveArmBackward() {
    ArmRotation.set(-1);
  }

  // stops the ArmRotation motor
  public void stopArmRotation() {
    ArmRotation.stopMotor();
  }
  public void MotorAngleForwards(double angle){
    if (angle > 270) {
      // if the angle is greater than 270, stop the motor(dont set to 0, then what other value?)
      if (toplimitSwitch.get()){
       ArmRotation.set(.01);
      }else{
        // if the angle is below that, then allow motor to keep moving
        ArmRotation.set(1);
      }
    
    }
    if (angle < 0) {
      // if the angle is less than 0, slow down/stop the motor (decide what value to use)
      if(bottomlimitSwitch.get()){
        ArmRotation.set(.01);
      
      }else{
        //if the angle is above that, then allow the motor to keep moving
        ArmRotation.set(1);
      }
    }
        
      }
    
    
  
  
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
