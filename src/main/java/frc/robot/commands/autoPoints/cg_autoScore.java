// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.commands.armAction.ca_moveArmToMiddleCompressor;
import frc.robot.commands.armAction.ca_moveToCarryCompressor;
import frc.robot.commands.armAction.ca_moveToRedzoneCompressor;
import frc.robot.commands.armAction.ca_setArmPosition;
import frc.robot.commands.armAction.ca_setSideOrientation;
import frc.robot.commands.armAction.cm_GripperClose;
import frc.robot.commands.armAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_doesAbsolutelyNothing;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cg_autoScore extends SequentialCommandGroup {
  /** Creates a new cg_autoScore. */

  public cg_autoScore(Drivetrain drivetrain, Arm arm, Gripper gripper) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new cm_GripperClose(gripper).withTimeout(0.5), // Closes on game piece
        new ca_moveToRedzoneCompressor(arm),
        new ca_moveArmToMiddleCompressor(arm),
        new cm_GripperOpen(gripper)
    );
  }
}
