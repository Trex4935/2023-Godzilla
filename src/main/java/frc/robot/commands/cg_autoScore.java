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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ArmRotation;
import frc.robot.extensions.ArmPosition;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Gripper;
import frc.robot.commands.ca_autoTurnKinematic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_autoScore extends SequentialCommandGroup {
  private TrajectoryConfig trajectoryConfig;
  /** Creates a new cg_autoScore. */
  
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(0, 0.25),
      new Translation2d(0, 0.5)),
    //new Translation2d(xn, yn),
    new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

  public cg_autoScore(Drivetrain drivetrain, ArmRotation armRotation, ArmExtension armExtension, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new cm_GripperClose(gripper), // Closes on game piece
      new ca_setArmPosition(ArmPosition.MIDDLE),
      new cm_GripperOpen(gripper), // Drops the game piece
      new ca_autoTrajectoryKinematic(drivetrain, trajectory) // Moves for mobility points

    );
  }
}
