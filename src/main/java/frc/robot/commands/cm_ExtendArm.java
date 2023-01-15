// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtension;

public class cm_ExtendArm extends CommandBase {
  private final ArmExtension m_arm;
  private final XboxController m_XboxController;
  
  /** Creates a new cm_ExtendArm. */
  public cm_ExtendArm(ArmExtension armSubsystem,XboxController XboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = armSubsystem;
    m_XboxController = XboxController;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.extendArm(m_XboxController.getRawAxis(4));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
