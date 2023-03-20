// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.armAction.ca_moveArmToMiddleCompressor;
import frc.robot.commands.armAction.ca_moveToCarryCompressor;
import frc.robot.commands.armAction.ca_moveToRedzoneCompressor;
import frc.robot.commands.armAction.cm_GripperClose;
import frc.robot.commands.armAction.cm_GripperOpen;
import frc.robot.commands.autoDriveActions.ca_balanceToMobility;
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
        new cg_autoScore(drivetrain, arm, gripper),
        new ca_moveToCarryCompressor(arm),
      // Go to Charge Station
        new ca_moveToChargeStation(drivetrain), // Senses incline: doAuto = true
        new WaitCommand(0.2),
        new ca_autoBalanceNew(drivetrain), // Ends automatically if doAuto == false
        new ca_balanceToMobility(drivetrain)
    );
    // If doing balance, move back and balance.
  }
}
