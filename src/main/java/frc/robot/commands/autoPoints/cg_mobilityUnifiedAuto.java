// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armAction.ca_moveArmToMiddle;
import frc.robot.commands.armAction.ca_moveToCarryCompressor;
import frc.robot.commands.armAction.cm_GripperClose;
import frc.robot.commands.armAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_doesAbsolutelyNothing;
import frc.robot.commands.autoDriveActions.ca_mobilityToBalance;
import frc.robot.commands.autoDriveActions.ca_moveToMobility;
import frc.robot.commands.armAction.ca_rotateArmToMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_mobilityUnifiedAuto extends SequentialCommandGroup {
  /** Creates a new cg_mobilityUnifiedAuto. */
  public cg_mobilityUnifiedAuto(Arm arm, Gripper gripper, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Cone Score:
        // new cm_GripperClose(gripper).raceWith(new ca_rotateArmToMiddle(arm)),
        new cm_GripperClose(gripper).withTimeout(.1),
        new ca_rotateArmToMiddle(arm),
        new ca_moveArmToMiddle(arm),
        new cm_GripperOpen(gripper),
      // Go to Mobility, while moving arm to CompressorSide:
        new ca_moveToCarryCompressor(arm).raceWith(new ca_moveToMobility(drivetrain))
    );
    // If doing balance, move back and balance.
    if (Constants.doAutoBalance) {
      addCommands(
        new ca_mobilityToBalance(drivetrain),
        new ca_doesAbsolutelyNothing().withTimeout(.2),
        new ca_autoBalance(drivetrain)
      );
    }
  }
}
