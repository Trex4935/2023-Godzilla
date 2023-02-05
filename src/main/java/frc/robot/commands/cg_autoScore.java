// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ArmRotation;
import frc.robot.extensions.ArmPosition;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Gripper;
import frc.robot.TrajectoryContainer;
import frc.robot.commands.ca_autoTurnKinematic;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_autoScore extends SequentialCommandGroup {
  /** Creates a new cg_autoScore. */
  
  public cg_autoScore(Drivetrain drivetrain, ArmRotation armRotation, ArmExtension armExtension, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new cm_GripperClose(gripper), // Closes on game piece
      new ca_setArmPosition(ArmPosition.MIDDLE),
      new cm_GripperOpen(gripper), // Drops the game piece
      new ca_autoTrajectoryKinematic(drivetrain, TrajectoryContainer.trajectoryf) // Moves for mobility points

    );
  }
}
