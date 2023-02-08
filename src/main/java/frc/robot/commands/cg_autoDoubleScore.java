// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryContainer;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class cg_autoDoubleScore extends SequentialCommandGroup {

  /** Creates a new cg_autoDoubleScore. */
  public cg_autoDoubleScore(Drivetrain drivetrain, Arm arm, Gripper gripper) {

    // Use addRequirements() here to declare subsystem dependencies
    addCommands(
        new cm_GripperClose(gripper), // Closes on game piece
        new ca_setArmPosition(ArmPosition.MIDDLE),
        new cm_GripperOpen(gripper), // Drops the game piece
        new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
        new ca_autoTrajectoryKinematic(drivetrain, TrajectoryContainer.trajectoryf), // Moves to game piece
        new ca_setSideOrientation(ArmSideOrientation.BatterySide), // Changes the arm side
        new ca_setArmPosition(ArmPosition.LOW), // Moves arm position to prepare for getting the piece
        new cm_GripperClose(gripper), // Gets piece
        new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
        new ca_autoTrajectoryKinematic(drivetrain, TrajectoryContainer.trajectoryb), // Moves to scoring area
        new ca_setSideOrientation(ArmSideOrientation.CompressorSide), // Changes the arm side
        new ca_setArmPosition(ArmPosition.MIDDLE), // Sets arm position to middle
        new cm_GripperOpen(gripper), // Drops the game piece
        new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
        new ca_autoTrajectoryKinematic(drivetrain, TrajectoryContainer.trajectorym) // Moving to get mobility points

    );
  }
}