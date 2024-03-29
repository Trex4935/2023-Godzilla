// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoDriveActions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.Arm;

public class ca_moveToCarryBattery extends CommandBase {
  Arm m_arm;
  /** Creates a new ca_moveToCarryCompressor. */
  public ca_moveToCarryBattery(Arm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.retractArm();
    m_arm.setArmRotationSM(Constants.ArmCarryAngleBattery);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.selectedArmSideOrientation = ArmSideOrientation.BatterySide;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Arm.checkRotation2(Constants.ArmCarryAngleBattery);
  }
}