// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoDriveActions.ca_driveForwardInches;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_unifiedAuto extends SequentialCommandGroup {
  /** Creates a new cg_unifiedAuto. */
  public cg_unifiedAuto(Arm arm, Gripper gripper, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Cone Score:
         new cg_autoScore(drivetrain, arm, gripper),
        new ca_moveToCarryCompressor(arm),
        // goes into charge station straight on 30in
        new ca_driveForwardInches(drivetrain, 0.6, 0.0, 180.0) //,
    );
  }
}
