// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmRotationConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.extensions.*;

public class Arm extends SubsystemBase {
  CANSparkMax ArmRotation;
  WPI_TalonFX ArmExtendingMotor;
  WPI_TalonFX AEM;
  Encoder encoder;
  FlippedDIO limitSwitchBottom;
  FlippedDIO limitSwitchTop;

  /** Creates a new ArmRotation. */
  public Arm(int SparkMaxID, int TalonFXID) {
    //Arm Rotation
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);
    //Arm Extension
    ArmExtendingMotor = new WPI_TalonFX(TalonFXID);
    AEM = ArmExtendingMotor;


  }
  // note for me: gear ratio is 200:1; incorporate later
public void moveArm(){
  
}
// checks bottom limits and return

public boolean stopBottomLimits(){
  return limitSwitchBottom.get() || encoder.get() <= 4000;
}

public boolean extendToBottom(XboxController ctrl){
  extendArm(ctrl);
  stopBottomLimit();
  stopEncoderBottomLimit();
  return stopBottomLimits();
}

// If limit switch is triggered, then stop the motor from extending 
public void stopBottomLimit(){

  if (limitSwitchBottom.get()) {
    AEM.stopMotor();
  };



}

// If Encoder is negative then stop the motor from extending 
public void stopEncoderBottomLimit(){
  if (encoder.get() <= 4000){
    AEM.stopMotor();
  }
}


// move the motor depending on my joystick inputs
public void extendArm(XboxController xboxController) {
  AEM.set(xboxController.getRawAxis(1));
 
 }


 
 

  @Override
  public void periodic() {
    // This method will be called once per scheduled run
  }
}
