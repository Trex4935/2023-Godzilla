// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extensions.ArmPosition;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.Constants;
import frc.robot.Robot;

public class ca_ArmMovementCombo extends CommandBase {

  private final ArmExtension m_extend;
  private final ArmRotation m_rotate;

  /** Creates a new ca_ArmMovementCombo. */
  public ca_ArmMovementCombo(ArmExtension extend, ArmRotation rotate) {
    m_extend = extend;
    m_rotate = rotate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extend, rotate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (Robot.selectedArmPosition) {
      case HIGH:
        m_rotate.AutoArmRotation(Constants.ArmHighAngle);
        m_extend.AutoArmExtension(Constants.ArmHighDistance);
        
      default:
        m_rotate.AutoArmRotation(Constants.ArmCarryAngle);
        m_extend.AutoArmExtension(Constants.ArmCarryDistance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
