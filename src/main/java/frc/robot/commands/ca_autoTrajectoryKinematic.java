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

public class ca_autoTrajectoryKinematic extends CommandBase {
  Timer timer;
  Trajectory traj;
  State currState;
  Drivetrain dt;
  Double end;
  Double sign;
  
  /** Creates a new cm_autoTrajectory. */
  public ca_autoTrajectoryKinematic(Drivetrain drivetrain, Trajectory trajectory, Double endPoint) {
    timer = new Timer();
    traj = trajectory;
    currState =  new State(0,0,0, new Pose2d(new Translation2d(0,0),new Rotation2d(0)),0);
    dt = drivetrain;
    end = endPoint;
    sign = 1.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    if (end <= traj.getInitialPose().getY()) {

      sign  = - 1.0;
      
    } else {
      sign = 1.0;
      
    }

    System.out.println(traj.getInitialPose().getY());
    System.out.println(end);
    System.out.println(sign);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currState = traj.sample(timer.get());

    Double velocityTarget  = currState.velocityMetersPerSecond;

    // Rate is 0, because we are following a straight line, the speed varies depending of path, it follows a trapezoide curve.
    Double leftSpeedWheel  =  dt.getLeftSpeedKin(velocityTarget, 0);
    Double rightSpeedWheel =  dt.getRightpeedKin(velocityTarget, 0);
    dt.driveWithController(leftSpeedWheel * Math.signum(end), rightSpeedWheel * Math.signum(end));
    System.out.println(leftSpeedWheel);
    System.out.println(rightSpeedWheel);
    System.out.println(sign);
    //System.out.println("Time: "+ timer.get() + " Velocity: " + velocityTarget +
    //" Position: " + currState.poseMeters.getY() + " LeftSpeed: " + leftSpeedWheel + " RightSpeed: " + rightSpeedWheel + "Sign:" + sign);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( (currState.poseMeters.getY() > end*Math.signum(end) - 0.001  || currState.poseMeters.getY() > end*Math.signum(end) + 0.001) && (currState.velocityMetersPerSecond < 0.001 ||  currState.velocityMetersPerSecond > -0.001));
  }
  
}
