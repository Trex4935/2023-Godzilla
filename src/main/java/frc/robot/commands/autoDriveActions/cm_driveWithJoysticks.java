// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoDriveActions;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class cm_driveWithJoysticks extends CommandBase {
  private final Drivetrain m_Drivetrain;
  private final Joystick m_joystickLeft;
  private final Joystick m_joystickRight;
  


  /** Creates a new cm_driveWithJoysticks. */
  public cm_driveWithJoysticks(Drivetrain dt, Joystick joystickLeft, Joystick joystickRight) {
    m_Drivetrain = dt;
    m_joystickLeft = joystickLeft;
    m_joystickRight = joystickRight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //This command moves robot by calling the joysticks method and taking in the inputs of both joysticks 
  m_Drivetrain.setMaxSpeed(Constants.dtmaxspeed);
  m_Drivetrain.driveWithJoysticks(m_joystickLeft, m_joystickRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stops motor when not moving Joysticks
    m_Drivetrain.stopMotors();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
