// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryContainer;
import frc.robot.commands.armAction.ca_setArmPosition;
import frc.robot.commands.armAction.ca_setSideOrientation;
import frc.robot.commands.armAction.cm_GripperClose;
import frc.robot.commands.armAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_autoDriveStraightTrajKinGyroEncPID;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class cg_autoScoreMobilityBalance extends SequentialCommandGroup {

  /** Creates a new cg_autoDoubleScore. */
  public cg_autoScoreMobilityBalance(Drivetrain drivetrain, Arm arm, Gripper gripper) {

    // Use addRequirements() here to declare subsystem dependencies
    addCommands(

        new cm_GripperClose(gripper).withTimeout(1.5), // Closes on game piece
        new ca_setArmPosition(ArmPosition.MIDDLE).withTimeout(1), // Drops the
        new ca_setSideOrientation(ArmSideOrientation.CompressorSide).withTimeout(5),
        new ca_setArmPosition(ArmPosition.MIDDLE).withTimeout(2),
        new cm_GripperOpen(gripper).alongWith(new ca_setArmPosition(ArmPosition.MIDDLE)).withTimeout(0.5), // , // Resets

        new ca_setArmPosition(ArmPosition.CARRY).withTimeout(2),
        new ca_autoDriveStraightTrajKinGyroEncPID(drivetrain, TrajectoryContainer.trajectoryMobility,
            TrajectoryContainer.trajMobilityEnd, 0.0), // Moves to game piece
        new ca_autoDriveStraightTrajKinGyroEncPID(drivetrain, TrajectoryContainer.trajectoryBack,
            TrajectoryContainer.trajBackEnd, 0.0), // Moves to game piece
        new ca_autoBalance(drivetrain)
    );
  }
}