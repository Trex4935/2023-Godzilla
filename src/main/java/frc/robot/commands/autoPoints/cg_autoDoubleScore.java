// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryContainer;
import frc.robot.commands.autoArmAction.ca_setArmPosition;
import frc.robot.commands.autoArmAction.ca_setSideOrientation;
import frc.robot.commands.autoArmAction.cm_GripperClose;
import frc.robot.commands.autoArmAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_autoDriveStraightTrajKinGyroEncPID;
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

        new cm_GripperClose(gripper).withTimeout(2), // Closes on game piece
        // new ca_setArmPosition(ArmPosition.MIDDLE).withTimeout(1), // Drops the
        new ca_setSideOrientation(ArmSideOrientation.CompressorSide).withTimeout(5),
        new ca_setArmPosition(ArmPosition.MIDDLE).withTimeout(2),
        new cm_GripperOpen(gripper).alongWith(new ca_setArmPosition(ArmPosition.MIDDLE)).withTimeout(2),// , // Resets arm to default position
        new ca_setArmPosition(ArmPosition.CARRY).withTimeout(2),
        new ca_autoDriveStraightTrajKinGyroEncPID(drivetrain,TrajectoryContainer.trajectoryMobility,TrajectoryContainer.trajMobilityEnd, 0.0), // Moves to game piece
        new ca_autoDriveStraightTrajKinGyroEncPID(drivetrain,TrajectoryContainer.trajectoryBack,TrajectoryContainer.trajBackEnd, 0.0) // Moves to game piece
        // new ca_setSideOrientation(ArmSideOrientation.BatterySide,
    // true).withTimeout(3), // Changes the arm side
    // new ca_setArmPosition(ArmPosition.LOW).withTimeout(1), // Moves arm position
    // to prepare for getting the piece
    // new cm_GripperClose(gripper).withTimeout(0.5), // Gets piece
    // new ca_setArmPosition(ArmPosition.CARRY).withTimeout(1), // Resets arm to
    // default position
    // new ca_autoTrajectoryKinematicWithGyro(drivetrain,
    // TrajectoryContainer.trajectoryBack,
    // TrajectoryContainer.trajBackEnd, 0.0), // Moves to scoring area
    // new ca_setSideOrientation(ArmSideOrientation.CompressorSide, false), //
    // Changes the arm side
    // new ca_setArmPosition(ArmPosition.MIDDLE), // Sets arm position to middle
    // new cm_GripperOpen(gripper).alongWith(new
    // ca_setArmPosition(ArmPosition.MIDDLE)), // Drops the game piece
    // new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
    // new ca_autoTrajectoryKinematicWithGyro(drivetrain,
    // TrajectoryContainer.trajectoryMobility,
    // TrajectoryContainer.trajMobilityEnd, 0.0) // Moving to get mobility points

    );
  }
}