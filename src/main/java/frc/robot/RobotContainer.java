// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Arm;
// Commands
import frc.robot.commands.ca_ArmMovementCombo;
import frc.robot.commands.ca_ForwardHalfSpeed;
import frc.robot.commands.ca_autoBalance;
import frc.robot.commands.ca_autoDoubleScoreBalance;
import frc.robot.commands.ca_autoDriveStraightTrajKinGyroEncPID;
import frc.robot.commands.ca_autoTrajectory;
import frc.robot.commands.ca_autoTrajectoryKinematic;
import frc.robot.commands.ca_autoTrajectoryKinematicWithGyro;
import frc.robot.commands.ca_autoTurnKinematic;
import frc.robot.commands.ca_autoTurnKinematicGyro;
import frc.robot.commands.ca_doSimpleL;
import frc.robot.commands.ca_doesAbsolutelyNothing;
import frc.robot.commands.ca_driveAutoSquare;
import frc.robot.commands.ca_setArmPosition;
import frc.robot.commands.ca_setSideOrientation;
import frc.robot.commands.cg_autoDoubleScore;
import frc.robot.commands.cg_autoScore;
import frc.robot.commands.cg_autoScoreBalance;
import frc.robot.commands.cm_driveWithJoysticks;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.commands.cm_GripperClose;
import frc.robot.commands.cm_GripperOpen;
import frc.robot.commands.ca_ArmMovementCombo;
import frc.robot.commands.cm_setGamePieceType;
import frc.robot.commands.cm_manualDecreaseExtendTicks;
import frc.robot.commands.cm_manualAddExtendTicks;
import frc.robot.commands.cm_manualResetAddArm;
import frc.robot.commands.cm_manualRotateBattery;
import frc.robot.commands.cm_manualRotateCompressor;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.commands.cm_setSpeedLimit;

// Misc
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Robot Base Class
public class RobotContainer {

  // Declare Subsystems
  public final Drivetrain drivetrain;
  private final Arm arm;
  private final Gripper gripper;

  // Declare Commands
  private final cm_driveWithJoysticks driveWithJoysticks;
  private final cm_setSpeedLimit setSpeedLimitMax;
  private final cm_setSpeedLimit setSpeedLimitDefault;
  private final ca_ArmMovementCombo armMovementCombo;
  private final ca_setArmPosition setArmPositionHigh;
  private final ca_setArmPosition setArmPositionMiddle;
  private final ca_setArmPosition setArmPositionLow;
  private final cm_manualDecreaseExtendTicks manualDecreaseExtendTicks;
  private final cm_manualAddExtendTicks manualAddExtendTicks;
  private final cm_manualRotateBattery manualRotateBattery;
  private final cm_manualRotateCompressor manualRotateCompressor;
  private final cm_manualResetAddArm manualResetAddArm;

  private final cm_GripperClose gripperClose;
  private final cm_GripperOpen gripperOpen;
  private final ca_setSideOrientation setSideOrientationCompressor;
  private final ca_setSideOrientation setSideOrientationBattery;
  /** Sets the game piece type to CubeTrue */
  private final cm_setGamePieceType setGamePieceTypeCubeTrue;
  /** Sets the game piece type to CubeFalse */
  private final cm_setGamePieceType setGamePieceTypeCubeFalse;
  private final ca_doesAbsolutelyNothing nothingAtAll;
  private final ca_autoTrajectoryKinematic autoTrajectoryKinematic;
  private final ca_autoTrajectory autoTrajectory;
  private final ca_autoTurnKinematic autoTurnTrajectory;
  private final ca_autoTurnKinematicGyro autoTurnTrajectoryWithGyro;
  private final ca_driveAutoSquare autoSquare;
  private final ca_autoBalance autoBalance;
  private final cg_autoDoubleScore autoDoubleScore;
  private final ca_autoDoubleScoreBalance autoDoubleScoreBalance;
  private final ca_doSimpleL autoSimpleL;
  private final cg_autoScore autoScore;
  private final cg_autoScoreBalance autoScoreAndBalance;
  private final ca_autoTrajectoryKinematicWithGyro driveStraight;

  private final ca_autoDriveStraightTrajKinGyroEncPID autoStraightPID;

  private final ca_ForwardHalfSpeed forwardHalfSpeed;
  // private final SequentialCommandGroup autoDoubleScoreAndBalancing;
  // private final SequentialCommandGroup backwordAndAutoBalancing;
  // private final SequentialCommandGroup doLAndAutoBalancing;
  // private final SequentialCommandGroup autoDoubleScoreAndBalancing;

  // Declare Other
  private final Joystick m_JoystickLeft = new Joystick(Constants.leftJoystick);
  private final Joystick m_JoystickRight = new Joystick(Constants.rightJoystick);
  private final Joystick m_ArduinoController = new Joystick(Constants.controllerID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Create Subsystem objects
    drivetrain = new Drivetrain();
    arm = new Arm();
    gripper = new Gripper();
    drivetrain.resetGyro();

    // Create Command objects

    // Combo
    nothingAtAll = new ca_doesAbsolutelyNothing();
    armMovementCombo = new ca_ArmMovementCombo(arm);
    setArmPositionHigh = new ca_setArmPosition(ArmPosition.HIGH);
    setArmPositionMiddle = new ca_setArmPosition(ArmPosition.MIDDLE);
    setArmPositionLow = new ca_setArmPosition(ArmPosition.LOW);
    manualDecreaseExtendTicks = new cm_manualDecreaseExtendTicks(arm);
    manualAddExtendTicks = new cm_manualAddExtendTicks(arm);
    manualRotateBattery = new cm_manualRotateBattery(arm);
    manualRotateCompressor = new cm_manualRotateCompressor(arm);
    manualResetAddArm = new cm_manualResetAddArm(arm);
    autoDoubleScoreBalance = new ca_autoDoubleScoreBalance(drivetrain, arm, gripper);

    // Robot
    setSideOrientationCompressor = new ca_setSideOrientation(ArmSideOrientation.CompressorSide);
    setSideOrientationBattery = new ca_setSideOrientation(ArmSideOrientation.BatterySide);

    setGamePieceTypeCubeTrue = new cm_setGamePieceType("cube");
    setGamePieceTypeCubeFalse = new cm_setGamePieceType("cone");

    // Drivetrain
    driveWithJoysticks = new cm_driveWithJoysticks(drivetrain, m_JoystickLeft, m_JoystickRight);
    setSpeedLimitMax = new cm_setSpeedLimit(1.0);
    setSpeedLimitDefault = new cm_setSpeedLimit(0.75);

    // Gripper
    gripperOpen = new cm_GripperOpen(gripper);
    gripperClose = new cm_GripperClose(gripper);

    // Put the drive train sendable values onto the networktables / dashboard
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(gripper);

    // Auto Bench-Test

    autoSquare = new ca_driveAutoSquare(drivetrain, TrajectoryContainer.trajectoryFront,
        TrajectoryContainer.trajFrontEnd);

    // Going Backword-mobility.

    forwardHalfSpeed = new ca_ForwardHalfSpeed(drivetrain);

    autoTrajectory = new ca_autoTrajectory(drivetrain, TrajectoryContainer.pigeontraj,
        TrajectoryContainer.pigeontrajEnd);

    autoTrajectoryKinematic = new ca_autoTrajectoryKinematic(drivetrain, TrajectoryContainer.trajectoryMobility,
        TrajectoryContainer.trajMobilityEnd);

    driveStraight = new ca_autoTrajectoryKinematicWithGyro(drivetrain, TrajectoryContainer.trajectoryMobility,
        TrajectoryContainer.trajMobilityEnd, 0.0);

    autoStraightPID = new ca_autoDriveStraightTrajKinGyroEncPID(drivetrain, TrajectoryContainer.pigeontraj,
        TrajectoryContainer.pigeontrajEnd, 0.0);

    // Be able to turn

    autoTurnTrajectory = new ca_autoTurnKinematic(drivetrain, 0.0, -110.0); // testing 90 degree Turn;

    autoTurnTrajectoryWithGyro = new ca_autoTurnKinematicGyro(drivetrain, 0.0, 270.0); // testing 90 degree Turn;

    // Make a point & go Backword-mobility.

    autoScore = new cg_autoScore(drivetrain, arm, gripper);

    // Make 2 point and go Backword-mobility.

    autoDoubleScore = new cg_autoDoubleScore(drivetrain, arm, gripper);

    // Do autobalancing.

    autoBalance = new ca_autoBalance(drivetrain);

    // Scores middle and balances.

    autoScoreAndBalance = new cg_autoScoreBalance(drivetrain, arm, gripper);

    // Go backword and do autobalancing.

    // backwordAndAutoBalancing = new SequentialCommandGroup(autoTrajectory,
    // autoBalance);

    // Go in a simple L

    autoSimpleL = new ca_doSimpleL(drivetrain);

    // Go in a simple L shape and do auto balancing.

    // doLAndAutoBalancing = new SequentialCommandGroup(autoSimpleL, autoBalance);

    // Make 2 points and go in a simple L shape and do auto balancing.

    // autoDoubleScoreAndBalancing = new SequentialCommandGroup(autoDoubleScore,
    // autoSimpleL);

    // Follow L Path using point-map.

    // Make 2 points and go in a simple L shape using point map and do auto
    // balancing.

    // Add encoder for real length measures to auto.

    // Add PID to auo.

    // Configure the trigger bindings
    configureBindings();

  }

  /** Use to define trigger->command mappings. */
  private void configureBindings() {

    // Makes controller driving the default command
    drivetrain.setDefaultCommand(driveWithJoysticks);

    // Run arm movement combo and restart if it end
    arm.setDefaultCommand(armMovementCombo);

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // Increase Speed when pressing triggers.
    new JoystickButton(m_JoystickLeft, Constants.joystickTrigger).onTrue(setSpeedLimitMax).onFalse(setSpeedLimitDefault);
    new JoystickButton(m_JoystickRight, Constants.joystickTrigger).onTrue(setSpeedLimitMax).onFalse(setSpeedLimitDefault);

    // Arduino Controller Button Mapping
    // Arm Movement
    new JoystickButton(m_ArduinoController, Constants.groundButtonID).whileTrue(setArmPositionLow);
    new JoystickButton(m_ArduinoController, Constants.middleButtonID).whileTrue(setArmPositionMiddle);
    new JoystickButton(m_ArduinoController, Constants.highButtonID).whileTrue(setArmPositionHigh);
    // manual EXTENSION
    new JoystickButton(m_ArduinoController, Constants.ardJoystickUp).whileTrue(manualAddExtendTicks);
    new JoystickButton(m_ArduinoController, Constants.ardJoystickDown).whileTrue(manualDecreaseExtendTicks);
    // manual ROTATION
    new JoystickButton(m_ArduinoController, Constants.ardJoystickLeft).whileTrue(manualRotateCompressor);
    new JoystickButton(m_ArduinoController, Constants.ardJoystickRight).whileTrue(manualRotateBattery);
    // reset manual extension & rotation
    new JoystickButton(m_ArduinoController, Constants.highButtonID).onFalse(manualResetAddArm);
    new JoystickButton(m_ArduinoController, Constants.middleButtonID).onFalse(manualResetAddArm);
    new JoystickButton(m_ArduinoController, Constants.groundButtonID).onFalse(manualResetAddArm);

    // Toggle Switches
    new JoystickButton(m_ArduinoController, Constants.gamePieceID).onTrue(setGamePieceTypeCubeTrue)
        .onFalse(setGamePieceTypeCubeFalse);

    new JoystickButton(m_ArduinoController, Constants.robotSideID).onTrue(setSideOrientationCompressor)
        .onFalse(setSideOrientationBattery);

    new JoystickButton(m_ArduinoController, Constants.gripperID).whileTrue(gripperOpen).whileFalse(gripperClose);

    // operator.b().toggleOnTrue(Commands.startEnd(gripper::gripOpen,
    // gripper::gripClose, gripper));

    // new JoystickButton(m_JoystickLeft, 1).toggleOnTrue(autoBalance);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    // return autoBalance.withTimeout(15);
    return autoScoreAndBalance;

    // A command will be run in autonomous
    // return forwardHalfSpeed;
  }

  // Sendable override
  // Anything put here will be added to the network tables and thus can be added
  // to the dashboard / consumed by the LED controller
  public void initSendable(SendableBuilder builder) {

  }
}
