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

public class ca_autoDoubleScoreBalance extends SequentialCommandGroup {
  /** Creates a new ca_autoDoubleScoreBalance. */
  public ca_autoDoubleScoreBalance(Drivetrain drivetrain, Arm arm, Gripper gripper) {

    // Use addRequirements() here to declare subsystem dependencies.
     addCommands(
        new cm_GripperClose(gripper).withTimeout(1), // Closes on game piece
        new ca_setArmPosition(ArmPosition.MIDDLE),
        new ca_setSideOrientation(ArmSideOrientation.CompressorSide),
        new cm_GripperOpen(gripper).withTimeout(1), // Drops the game piece
        new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
        new ca_autoTrajectoryKinematicWithGyro(drivetrain, TrajectoryContainer.trajectoryFront,
            TrajectoryContainer.trajFrontEnd, 0.0), // Moves to game piece
        new ca_setSideOrientation(ArmSideOrientation.BatterySide).withTimeout(1), // Changes the arm side
        new ca_setArmPosition(ArmPosition.LOW), // Moves arm position to prepare for getting the piece
        new cm_GripperClose(gripper).withTimeout(1), // Gets piece
        new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
        new ca_autoTrajectoryKinematicWithGyro(drivetrain, TrajectoryContainer.trajectoryBack,
            TrajectoryContainer.trajBackEnd, 0.0), // Moves to scoring area
        new ca_setSideOrientation(ArmSideOrientation.CompressorSide), // Changes the arm side
        new ca_setArmPosition(ArmPosition.MIDDLE), // Sets arm position to middle
        new cm_GripperOpen(gripper), // Drops the game piece
        new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
        new ca_doSimpleL(drivetrain), // Autonomous
        new ca_autoBalance(drivetrain) // Balances the drivetrain
        );
  }

}
