// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotation;

public class ca_AutoArmRotationAngle extends CommandBase {

  private final ArmRotation m_arm;
  private final double m_angle;
  private boolean finCheck;

  /** Creates a new ca_AutoArmRotationAngle. */
  public ca_AutoArmRotationAngle(ArmRotation arm, double angle) {
    m_arm = arm;
    addRequirements(m_arm);
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finCheck = m_arm.AutoArmRotation(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stopArmRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finCheck;
  }
}
