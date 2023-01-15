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

public class ArmRotation extends SubsystemBase {
  CANSparkMax ArmRotation;

  /** Creates a new ArmRotation. */
  public ArmRotation() {
    // init motor
    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);

    // sets the speed that the arm moves forward ()
  }

  public void moveArmForward() {
    ArmRotation.set(1);
  }

  // sets the speed that the arm moves backward
  public void moveArmBackward() {
    ArmRotation.set(-1);
  }

  /*
   * Rotates the arm by using buttons
   * public void RotateArmWithButtons(XboxController xboxController){
   * 
   * }
   */
  // stops the ArmRotation motor
  public void stopArmRotation() {
    ArmRotation.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}