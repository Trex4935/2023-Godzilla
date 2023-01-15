// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.extensions.*;

public class ArmExtension extends SubsystemBase {
  private WPI_TalonFX ArmExtensionMotor;

  /** Creates a new motor. */
  public ArmExtension() {
    // Arm Extension
    ArmExtensionMotor = Falcon.createDefaultFalcon(ArmConstants.armExtensionCAN);
    /// Need Encoder based soft limits implemented
  }

  // Stop the Extension motor
  public void stopExtensionMotor() {
    ArmExtensionMotor.stopMotor();
  }

  // Takes a speed values from -1 to 1 and set the extension motor to that value
  public void extendArm(Double speed) {
    ArmExtensionMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
