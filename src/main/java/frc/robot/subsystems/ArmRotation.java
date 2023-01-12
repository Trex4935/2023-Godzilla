// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    ArmRotation = SparkMax.createDefaultCANSparkMax(ArmRotationConstants.armRotationCAN);

  }
  // note for me: gear ratio is 200:1; incorporate later
public void moveArm(){
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
