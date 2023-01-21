// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ArmRotation;

// Commands
import frc.robot.commands.cm_armRotationForward;
import frc.robot.Constants.MovementConstraints;
import frc.robot.commands.ca_ForwardHalfSpeed;
import frc.robot.commands.ca_autoTrajectory;
import frc.robot.commands.cm_armRotationBackward;
import frc.robot.commands.cm_driveWithJoysticks;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// Other
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Robot Base Class
public class RobotContainer {

  // Declare Subsystems
  private final Drivetrain drivetrain;
  private final ArmExtension armextension;
  private final ArmRotation armrotation;

  // Declare Commands
  private final cm_driveWithJoysticks driveWithJoysticks;
  // private final cm_ExtendArm extendArm;
  private final cm_armRotationForward armRotationForward;
  private final cm_armRotationBackward armRotationBackward;
  private final ca_ForwardHalfSpeed forwardHalfSpeed;
  private final ca_autoTrajectory autoTrajectory;

  // Declare Other
  private final Joystick m_JoystickLeft = new Joystick(0);
  private final Joystick m_JoystickRight = new Joystick(1);
  private CommandXboxController operator = new CommandXboxController(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Create Subsystem objects
    drivetrain = new Drivetrain();
    armextension = new ArmExtension();
    armrotation = new ArmRotation();

    // Create Command objects
    // extendArm = new cm_ExtendArm(armextension, 0.0);
    armRotationForward = new cm_armRotationForward(armrotation);
    armRotationBackward = new cm_armRotationBackward(armrotation);
    driveWithJoysticks = new cm_driveWithJoysticks(drivetrain, m_JoystickLeft, m_JoystickRight);
    forwardHalfSpeed = new ca_ForwardHalfSpeed(drivetrain);

    // Put the drive train sendable values onto the networktables / dashboard
    SmartDashboard.putData(drivetrain);


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      MovementConstraints.dtmaxspeed, MovementConstraints.dtmaxaccel);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0, 0.25),
        new Translation2d(0, 0.5)),
        //new Translation2d(xn, yn),
      new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
      trajectoryConfig);

      autoTrajectory = new ca_autoTrajectory(drivetrain, trajectory);


    // Configure the trigger bindings
    configureBindings();

  }

  /** Use to define trigger->command mappings. */
  private void configureBindings() {

    // Makes controller driving the default command
    drivetrain.setDefaultCommand(driveWithJoysticks);

    // Creates command to move the arm in and out
    new RunCommand(() -> armextension.extendArm(operator.getRightX()));

    
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    operator.b().whileTrue(armRotationForward);
    operator.x().whileTrue(armRotationBackward);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    
      

      //Fill in once we have more info/constants
      /* RamseteCommand ramseteCommand =
      new RamseteCommand(
          trajectory,
          drivetrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivetrain::tankDriveVolts,
          drivetrain);

    drivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
 */

    return autoTrajectory;

    // A command will be run in autonomous
    //return forwardHalfSpeed;
  }

}
