// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class TrajectoryContainer {

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.dtmaxspeed, Constants.dtmaxaccel);

        
    public static final Trajectory pigeontraj = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
        new Translation2d(0, 0.25),
        new Translation2d(0, 0.5)),
    //new Translation2d(xn, yn),
    new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

    public static final Trajectory trajectoryf = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(),
    new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

    public static final Trajectory trajectoryb = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 1, new Rotation2d(0)),
    List.of(),
    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

    public static final Trajectory trajectorym = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(),
    new Pose2d(0, 0.8, Rotation2d.fromDegrees(0)),
    trajectoryConfig);

}
