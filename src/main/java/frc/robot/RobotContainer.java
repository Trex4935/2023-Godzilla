// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Gripper;

// Commands
import frc.robot.commands.cm_armRotationForward;
import frc.robot.commands.ca_ArmMovementCombo;
import frc.robot.commands.ca_AutoArmExtensionDistance;
import frc.robot.commands.ca_AutoArmRotationAngle;
import frc.robot.commands.ca_ForwardHalfSpeed;
import frc.robot.commands.ca_autoTrajectory;
import frc.robot.commands.ca_autoTrajectoryKinematic;
import frc.robot.commands.ca_autoTurnKinematic;
import frc.robot.commands.ca_driveAutoSquare;
import frc.robot.commands.ca_setArmPosition;
import frc.robot.commands.cm_armRotationBackward;
import frc.robot.commands.cm_driveWithJoysticks;
import frc.robot.extensions.ArmPosition;
import frc.robot.commands.cm_GripperClose;
import frc.robot.commands.cm_GripperOpen;
import frc.robot.commands.ca_ArmMovementCombo;

// Misc
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Robot Base Class
public class RobotContainer {

  // Declare Subsystems
  private final Drivetrain drivetrain;
  private final ArmExtension armextension;
  private final ArmRotation armrotation;
  private final Gripper gripper;

  // Declare Commands
  private final cm_driveWithJoysticks driveWithJoysticks;
  private final ca_ArmMovementCombo armMovementCombo;
  private final ca_setArmPosition setArmPositionHigh;
  private final ca_setArmPosition setArmPositionMiddle;
  private final ca_setArmPosition setArmPositionLow;

  private final cm_GripperClose gripperClose;
  private final cm_GripperOpen gripperOpen;
  private final ca_autoTrajectoryKinematic autoTrajectoryKinematic;
  private final ca_autoTrajectory autoTrajectory;
  private final ca_autoTurnKinematic autoTurnTrajectory;
  private final ca_driveAutoSquare autoSquare;

  // Declare Other
  private final Joystick m_JoystickLeft = new Joystick(Constants.joystickLeft);
  private final Joystick m_JoystickRight = new Joystick(Constants.joystickRight);
  private CommandXboxController operator = new CommandXboxController(Constants.controllerID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Create Subsystem objects
    drivetrain = new Drivetrain();
    armextension = new ArmExtension();
    armrotation = new ArmRotation();
    gripper = new Gripper();

    // Create Command objects
    
    //Combo
    armMovementCombo = new ca_ArmMovementCombo(armextension, armrotation);
    setArmPositionHigh = new ca_setArmPosition(ArmPosition.HIGH);
    setArmPositionMiddle = new ca_setArmPosition(ArmPosition.MIDDLE);
    setArmPositionLow = new ca_setArmPosition(ArmPosition.LOW);
    
    // Drivetrain
    driveWithJoysticks = new cm_driveWithJoysticks(drivetrain, m_JoystickLeft, m_JoystickRight);
    
    // Gripper
    gripperOpen = new cm_GripperOpen(gripper);
    gripperClose = new cm_GripperClose(gripper);

    // Put the drive train sendable values onto the networktables / dashboard
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(armextension);
    SmartDashboard.putData(armrotation);
    SmartDashboard.putData(gripper);

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.dtmaxspeed, Constants.dtmaxaccel);

        
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(0, 0.25),
          new Translation2d(0, 0.5)),
        //new Translation2d(xn, yn),
        new Pose2d(0, 1, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

      autoTrajectoryKinematic = new ca_autoTrajectoryKinematic(drivetrain, trajectory);
      autoTurnTrajectory = new ca_autoTurnKinematic(drivetrain, 0.0, - 135.0); // testing 90 degree Turn;
      autoSquare = new ca_driveAutoSquare(drivetrain, trajectory);


    autoTrajectory = new ca_autoTrajectory(drivetrain, trajectory);

    // Configure the trigger bindings
    configureBindings();

  }

  /** Use to define trigger->command mappings. */
  private void configureBindings() {

    // Makes controller driving the default command
    drivetrain.setDefaultCommand(driveWithJoysticks);

    // Run arm movement combo and restart if it ends
    armMovementCombo.repeatedly();

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // Adjust the Combo state machine to High/Middle/Low based on button press
    operator.y().whileTrue(setArmPositionHigh);
    operator.x().whileTrue(setArmPositionMiddle);
    operator.a().whileTrue(setArmPositionLow);


    operator.b().toggleOnTrue(Commands.startEnd(gripper::gripOpen, gripper::gripClose, gripper));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    
    // Fill in once we have more info/constants
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

    return autoSquare;

    // A command will be run in autonomous
    // return forwardHalfSpeed;
  }

}
