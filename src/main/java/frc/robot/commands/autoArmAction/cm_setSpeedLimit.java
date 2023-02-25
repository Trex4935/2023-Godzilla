// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoArmAction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class cm_setSpeedLimit extends CommandBase {
  double SpeedLimit;
  /** Creates a new cm_releaseSpeedLimiter. */
  public cm_setSpeedLimit(double m_speedLimit) {
    SpeedLimit = m_speedLimit;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Constants.speedLimit = SpeedLimit;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
