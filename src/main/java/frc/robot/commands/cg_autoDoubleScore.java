// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class cg_autoDoubleScore extends SequentialCommandGroup {
  private TrajectoryConfig trajectoryConfig;

  Trajectory trajectoryf = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(0, 0.25),
      new Translation2d(0, 0.5)),
    //new Translation2d(xn, yn),
    new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

    Trajectory trajectoryb = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 1, new Rotation2d(0)),
    List.of(
      new Translation2d(0, 0.5),
      new Translation2d(0, 0.25)),
    //new Translation2d(xn, yn),
    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

    Trajectory trajectorym = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(0, 0.2),
      new Translation2d(0, 0.4)),
    //new Translation2d(xn, yn),
    new Pose2d(0, 0.8, Rotation2d.fromDegrees(0)),
    trajectoryConfig);


  /** Creates a new cg_autoDoubleScore. */
  public cg_autoDoubleScore(Drivetrain drivetrain, ArmRotation armRotation, ArmExtension armExtension, Gripper gripper) {

    // Use addRequirements() here to declare subsystem dependencies
    addCommands(
      new cm_GripperClose(gripper), // Closes on game piece
      new ca_setArmPosition(ArmPosition.MIDDLE),
      new cm_GripperOpen(gripper), // Drops the game piece
      new ca_setArmPosition(ArmPosition.CARRY), //Resets arm to default position
      new ca_autoTrajectoryKinematic(drivetrain, trajectoryf), // Moves to game piece
      new ca_setSideOrientation(ArmSideOrientation.BatterySide), //  Changes the arm side
      new ca_setArmPosition(ArmPosition.LOW), // Moves arm position to prepare for getting the piece
      new cm_GripperClose(gripper), //  Gets piece
      new ca_setArmPosition(ArmPosition.CARRY), //Resets arm to default position
      new ca_autoTrajectoryKinematic(drivetrain, trajectoryb), // Moves to scoring area
      new ca_setSideOrientation(ArmSideOrientation.CompressorSide), // Changes the arm side
      new ca_setArmPosition(ArmPosition.MIDDLE), //Sets arm position to middle
      new cm_GripperOpen(gripper), // Drops the game piece
      new ca_setArmPosition(ArmPosition.CARRY), // Resets arm to default position
      new ca_autoTrajectoryKinematic(drivetrain, trajectorym) // Moving to get mobility points





    );
  }
}