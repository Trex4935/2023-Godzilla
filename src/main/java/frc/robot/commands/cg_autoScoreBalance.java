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

public class cg_autoScoreBalance extends SequentialCommandGroup {

  /** Creates a new cg_autoDoubleScore. */
  public cg_autoScoreBalance(Drivetrain drivetrain, Arm arm, Gripper gripper) {

    // Use addRequirements() here to declare subsystem dependencies
    addCommands(

        new cm_GripperClose(gripper).withTimeout(0.75), // Closes on game piece
        new ca_setSideOrientation(ArmSideOrientation.CompressorSide).withTimeout(6),
        new ca_setArmPosition(ArmPosition.MIDDLE),
        new cm_GripperOpen(gripper).alongWith(new ca_setArmPosition(ArmPosition.MIDDLE)), // , // Resets arm to default
                                                                                          // position
        new ca_doesAbsolutelyNothing().withTimeout(1),
        new ca_autoTrajectoryKinematicWithGyro(drivetrain, TrajectoryContainer.trajectoryFront,
            TrajectoryContainer.trajFrontEnd, 0.0), // Moves to game piece
        // new ca_autoTrajectoryKinematicWithGyro(drivetrain,
        // TrajectoryContainer.trajectoryBack,TrajectoryContainer.trajBackEnd, 0.0), //
        // Moves to game piece
        new ca_autoBalance(drivetrain) // Balances the drivetrain
    );
  }
}