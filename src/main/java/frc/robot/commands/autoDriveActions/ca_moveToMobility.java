// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoDriveActions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ca_moveToMobility extends CommandBase {
  Drivetrain m_drivetrain;
  private int i = 0;
  /** Creates a new ca_driveMobility. */
  public ca_moveToMobility(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveStraightTarget(Constants.autoSpeed, Constants.autoAngle, Constants.autoMobilityPosition);
    
    if (m_drivetrain.checkPitch()) {
      i++;
      if (i > 10) {
        Constants.doAutoBalance = true;
      }
    } else {
      i = 0;  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.reachDriveTarget(Constants.autoMobilityPosition);
  }
}
