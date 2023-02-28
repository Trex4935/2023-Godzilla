// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armAction;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;

public class cm_setArmPositionManual extends CommandBase {
  ArmPosition m_armState;
  Timer timer;

  /** Creates a new ca_setArmPosition. */
  public cm_setArmPositionManual(ArmPosition armState) {
    m_armState = armState; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Constants.buttonOccupied = true;
    Constants.selectedArmState = m_armState;
      //encoder values of rotation compared to target ticks for encoder position to turn true
    // System.out.println(m_armState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.buttonOccupied = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
