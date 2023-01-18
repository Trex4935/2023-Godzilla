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
    if (toplimitSwitch.get() == true) {
      // if the toplimitSwitch is true, stop the motors
      if (toplimitSwitch.get()){
       ArmRotation.set(.01);
      }else{
        // if the toplimitSwitch is false, then allow motor to keep moving
        ArmRotation.set(1);
      }
    }
    
  }

  // sets the speed that the arm moves backward
  public void moveArmBackward() {
    ArmRotation.set(-1);
    if ( bottomlimitSwitch.get() == true) {
      // if the bottomlimitSwitch is true,stop the motor 
      if(bottomlimitSwitch.get()){
        ArmRotation.set(.01);
      
      }else{
        //if the bottomlimitSwitch is false, then allow the motor to keep moving
        ArmRotation.set(1);
      }
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
