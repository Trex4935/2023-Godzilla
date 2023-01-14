// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
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

public void extendArm(XboxController xboxController) {
  if (limitSwitchTop.get() == false)
 AEM.set(xboxController.getRawAxis(1));
 else {AEM.stopMotor();}

 if (limitSwitchBottom.get() == false)
 AEM.set(xboxController.getRawAxis(0));
 else {AEM.stopMotor();}
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
