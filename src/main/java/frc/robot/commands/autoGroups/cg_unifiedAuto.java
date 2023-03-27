// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoDriveActions.ca_driveForwardInches;
import frc.robot.commands.autoDriveActions.ca_driveBackwardInches;
import frc.robot.commands.autoDriveActions.ca_moveToCarryBattery;
import frc.robot.commands.autoDriveActions.ca_moveToCarryCompressor;
import frc.robot.commands.teleop.ca_moveArmToGround;
import frc.robot.commands.teleop.cm_GripperClose;
import frc.robot.commands.teleop.cm_GripperOpen;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_unifiedAuto extends SequentialCommandGroup {
  /** Creates a new cg_unifiedAuto. */
  public cg_unifiedAuto(Arm arm, Gripper gripper, Drivetrain drivetrain, Double constants) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Cone Score:
         new cg_autoScore(drivetrain, arm, gripper),
         new ca_moveToCarryCompressor(arm), 
        // goes to the second piece to pick up
        new ca_driveForwardInches(drivetrain).alongWith(new ca_moveToCarryBattery(arm)),
        // arm goes to ground, picks up a piece, and goes back to score it
        new ca_moveArmToGround(arm),
        new cm_GripperOpen(gripper).withTimeout(.5),
        new cm_GripperClose(gripper).alongWith(new ca_driveBackwardInches(drivetrain)),
        new cg_autoScore(drivetrain, arm, gripper)
    );
  }
}
