// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ca_moveToChargeStation extends CommandBase {

  private final Drivetrain drivetrain;

  /** Creates a new fastAutoBalance. */
  public ca_moveToChargeStation(Drivetrain dt) {

    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets Speed, Position and angle of where to go to -for mobility points-
    drivetrain.driveStraightTarget(Constants.autoSpeed,  Constants.autoAngle, Constants.autoPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  //stops checking for pitch change once the pitch has been triggered to start autobalance code.
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  //Boolean to check whether the pitch of the robot has change in order or indicate for the autobalance ot trigger.
  public boolean isFinished() {
    System.out.println(drivetrain.checkPitch());
    return drivetrain.reachDriveTarget(Constants.autoPosition);//drivetrain.checkPitch() || 
  }
}
