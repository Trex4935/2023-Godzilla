// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.subsystems.Arm;

public class ca_setArmPosition extends CommandBase {
  ArmPosition m_armState;
  Timer timer;

  /** Creates a new ca_setArmPosition. */
  public ca_setArmPosition(ArmPosition armState) {
    m_armState = armState; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Constants.selectedArmState = m_armState;
      //encoder values of rotation compared to target ticks for encoder position to turn true
    // System.out.println(m_armState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Arm.checkRotation(m_armState) && Arm.checkExtension(m_armState);
  }
}
