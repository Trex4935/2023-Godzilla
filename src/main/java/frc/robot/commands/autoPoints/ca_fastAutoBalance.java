// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoPoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ca_fastAutoBalance extends CommandBase {

  private final Drivetrain drivetrain;

  /** Creates a new fastAutoBalance. */
  public ca_fastAutoBalance(Drivetrain dt) {

    drivetrain = dt;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveStraightTarget(Constants.autoSpeed,  Constants.autoAngle, Constants.autoPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(drivetrain.checkPitch());
    return drivetrain.checkPitch();
  }
}
