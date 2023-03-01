// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armAction.ca_setArmPosition;
import frc.robot.commands.armAction.ca_setSideOrientation;
import frc.robot.commands.armAction.cm_GripperClose;
import frc.robot.commands.armAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_doSimpleL;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class ca_coneDropLconeTwoPrePosition extends SequentialCommandGroup {
  /** Creates a new ca_coneDropLconeTwoPrePosition. */
  public ca_coneDropLconeTwoPrePosition(Drivetrain drivetrain, Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
    new cm_GripperClose(gripper).withTimeout(1), // Closes on game piece
    new ca_setArmPosition(ArmPosition.MIDDLE).withTimeout(1),
    new ca_setSideOrientation(ArmSideOrientation.CompressorSide), // Changes the arm side
    new cm_GripperOpen(gripper), // Drops the game piece
    new ca_doSimpleL(drivetrain), // Autonomous
    new ca_doSimpleL(drivetrain), // Autonomous
    new ca_setArmPosition(ArmPosition.MIDDLE).withTimeout(1),
    new ca_setSideOrientation(ArmSideOrientation.CompressorSide), // Changes the arm side
    new cm_GripperOpen(gripper) // Drops the game piece
    );
  }

}