// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoDriveActions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ca_driveBackwardInches extends CommandBase {
  private Drivetrain m_drivetrain;
  private Double m_angle;
  private Double m_finalDistance;
  private Double m_speed;
  private boolean check;
  /** Creates a new ca_driveMobility. */
  public ca_driveBackwardInches(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    check = m_drivetrain.driveStraightTarget(0.6, 0, -Constants.autoTwoPiece);
  }
  // 0.7 < 0.8 < 0.9

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check;
  }
}
