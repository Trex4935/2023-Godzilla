// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoDriveActions;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ca_driveForwardInches extends CommandBase {
  Drivetrain m_drivetrain;
  Double m_angle;
  Double m_finalDistance;
  /** Creates a new ca_driveMobility. */
  public ca_driveForwardInches(Drivetrain drivetrain, Double angle, Double finalDistance) {
    m_drivetrain = drivetrain;
    m_angle = angle;
    m_finalDistance = finalDistance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveStraightTarget(0.8, m_angle, Units.inchesToMeters(m_finalDistance));
  }
  // 0.7 < 0.8 < 0.9

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.reachDriveTarget(Units.inchesToMeters(m_finalDistance)); 
  }
}
