// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

// Going straight autonomous using only trajectory, kinematics, gyro, encoders and timer + 2 PIDs.
public class ca_autoDriveStraightTrajKinGyroEncPID extends CommandBase {
  Timer timer;
  Trajectory traj;
  State currState;
  Drivetrain dt;
  Double end;

  /** Creates a new cm_autoTrajectory. */
  public ca_autoDriveStraightTrajKinGyroEncPID(Drivetrain drivetrain, Trajectory trajectory, Double endPoint) {
    timer = new Timer();
    traj = trajectory;
    currState = new State(0, 0, 0, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 0);
    dt = drivetrain;
    end = endPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currState = traj.sample(timer.get());
    Double time  = timer.get();
    dt.driveWithPIDArcade(currState, end, time);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return dt.reachDriveTarget(end);

}
}
