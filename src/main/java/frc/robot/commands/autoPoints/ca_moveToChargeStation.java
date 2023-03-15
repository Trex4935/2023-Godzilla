// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ca_moveToChargeStation extends CommandBase {

  private final Drivetrain m_drivetrain;
  private int i = 0;


  /** Creates a new fastAutoBalance. */
  public ca_moveToChargeStation(Drivetrain drivetrain) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets Speed, Position and angle of where to go to -for mobility points-
    m_drivetrain.driveStraightTarget(Constants.autoSpeed, Constants.autoAngle, Constants.autoChargeStationPosition);
    
    // Check if on incline for more than 200ms (10 cycles).
    if (m_drivetrain.checkPitch()) {
      i++;
      if (i > 10) {
        // If on incline for longer than 200ms, then perform balance.
        Constants.doAutoBalance = true;
      }
    } else {
      i = 0;  
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  //stops checking for pitch change once the pitch has been triggered to start autobalance code.
  public void end(boolean interrupted) {
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  //Boolean to check whether the pitch of the robot has change in order or indicate for the autobalance ot trigger.
  public boolean isFinished() {
    /* System.out.println(m_drivetrain.checkPitch()); */
    return m_drivetrain.reachDriveTarget(Constants.autoChargeStationPosition);//drivetrain.checkPitch() || 
  }
}
