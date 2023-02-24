// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.extensions.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
public class ca_pickUpFromGround extends SequentialCommandGroup {
  
  /** Creates a new ca_pickUpFromGround. */
  public ca_pickUpFromGround(Arm arm, Gripper gripper) {
    addCommands(
      //2 is a placeholder value, set to change !!!
      new ca_setArmPosition(ArmPosition.LOW).withTimeout(2),
      new cm_GripperClose(gripper).withTimeout(2),
      new ca_setArmPosition(ArmPosition.CARRY)
      
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

}
