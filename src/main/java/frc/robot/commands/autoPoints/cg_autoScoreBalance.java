// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armAction.ca_moveArmToMiddleCompressor;
import frc.robot.commands.armAction.ca_moveToRedzoneCompressor;
import frc.robot.commands.armAction.ca_rotateArmToMiddle;
import frc.robot.commands.armAction.cm_GripperClose;
import frc.robot.commands.armAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_doesAbsolutelyNothing;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class cg_autoScoreBalance extends SequentialCommandGroup {

  /** Creates a new cg_autoDoubleScore. */
  public cg_autoScoreBalance(Drivetrain drivetrain, Arm arm, Gripper gripper) {

    // Use addRequirements() here to declare subsystem dependencies
    addCommands(
      // Cone Score:
        // new cm_GripperClose(gripper).raceWith(new ca_rotateArmToMiddle(arm)),
        new cm_GripperClose(gripper).withTimeout(.1),
        new ca_rotateArmToMiddle(arm),
        new ca_moveArmToMiddleCompressor(arm),
        new cm_GripperOpen(gripper),
      // Go to Balance, while moving arm to CompressorSide:
        new ca_moveToChargeStation(drivetrain),
        new ca_doesAbsolutelyNothing().withTimeout(.2),
        new ca_autoBalance(drivetrain)
    );
  }
}