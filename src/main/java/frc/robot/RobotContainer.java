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
import frc.robot.commands.ca_autoTrajectory;
import frc.robot.commands.ca_autoTrajectoryKinematic;
import frc.robot.commands.ca_autoTurnKinematic;
import frc.robot.commands.ca_driveAutoSquare;
import frc.robot.commands.ca_setArmPosition;
import frc.robot.commands.ca_setSideOrientation;
import frc.robot.commands.cg_autoDoubleScore;
import frc.robot.commands.cg_autoScore;
import frc.robot.commands.cm_driveWithJoysticks;
import frc.robot.commands.cm_moveArmBattery;
import frc.robot.commands.cm_moveArmCompressor;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.commands.cm_GripperClose;
import frc.robot.commands.cm_GripperOpen;
import frc.robot.commands.ca_ArmMovementCombo;
import frc.robot.commands.cm_setGamePieceType;
import frc.robot.commands.cm_moveArmLeft;
import frc.robot.commands.cm_moveArmRight;
import frc.robot.commands.cm_decreaseExtensionTicks;
import frc.robot.commands.cm_increaseExtensionTicks;
import frc.robot.commands.cm_resetExtensionTicks;

// Misc
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Robot Base Class
public class RobotContainer {

  // Declare Subsystems
  private final Drivetrain drivetrain;
  private final Arm arm;
  private final Gripper gripper;

  // Declare Commands
  private final cm_driveWithJoysticks driveWithJoysticks;
  private final ca_ArmMovementCombo armMovementCombo;
  private final ca_setArmPosition setArmPositionHigh;
  private final ca_setArmPosition setArmPositionMiddle;
  private final ca_setArmPosition setArmPositionLow;
  private final cm_decreaseExtensionTicks decreaseExtensionTicks;
  private final cm_increaseExtensionTicks increaseExtensionTicks;
  private final cm_resetExtensionTicks resetExtensionTicks;
  
  // __________________________

  private final cm_moveArmCompressor moveArmCompressor;
  private final cm_moveArmBattery moveArmBattery;
  private final cm_moveArmLeft moveArmLeft;
  private final cm_moveArmRight moveArmRight;

  // __________________________

  private final cm_GripperClose gripperClose;
  private final cm_GripperOpen gripperOpen;
  private final ca_setSideOrientation setSideOrientationCompressor;
  private final ca_setSideOrientation setSideOrientationBattery;
  /** Sets the game piece type to CubeTrue */
  private final cm_setGamePieceType setGamePieceTypeCubeTrue;
  /** Sets the game piece type to CubeFalse */
  private final cm_setGamePieceType setGamePieceTypeCubeFalse;
  private final ca_autoTrajectoryKinematic autoTrajectoryKinematic;
  private final ca_autoTrajectory autoTrajectory;
  private final ca_autoTurnKinematic autoTurnTrajectory;
  private final ca_driveAutoSquare autoSquare;
  private final cg_autoDoubleScore autoDoubleScore;
  private final cg_autoScore autoScore;

  // Declare Other
  private final Joystick m_JoystickLeft = new Joystick(Constants.LeftJoystickX);
  private final Joystick m_JoystickRight = new Joystick(Constants.LeftJoystickY);
  private final Joystick m_ArduinoController = new Joystick(Constants.controllerID);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Create Subsystem objects
    drivetrain = new Drivetrain();
    arm = new Arm();
    gripper = new Gripper();

    // Create Command objects

    // __________________________

    moveArmCompressor = new cm_moveArmCompressor(arm);
    moveArmBattery = new cm_moveArmBattery(arm);
    moveArmLeft = new cm_moveArmLeft(arm);
    moveArmRight = new cm_moveArmRight(arm);

    // __________________________

    // Combo
    armMovementCombo = new ca_ArmMovementCombo(arm);
    setArmPositionHigh = new ca_setArmPosition(ArmPosition.HIGH);
    setArmPositionMiddle = new ca_setArmPosition(ArmPosition.MIDDLE);
    setArmPositionLow = new ca_setArmPosition(ArmPosition.LOW);
    decreaseExtensionTicks = new cm_decreaseExtensionTicks(arm);
    increaseExtensionTicks = new cm_increaseExtensionTicks(arm);
    resetExtensionTicks = new cm_resetExtensionTicks(arm);

    // Robot
    setSideOrientationCompressor = new ca_setSideOrientation(ArmSideOrientation.CompressorSide);
    setSideOrientationBattery = new ca_setSideOrientation(ArmSideOrientation.BatterySide);

    setGamePieceTypeCubeTrue = new cm_setGamePieceType(gripper, true);
    setGamePieceTypeCubeFalse = new cm_setGamePieceType(gripper, false);

    // Drivetrain
    driveWithJoysticks = new cm_driveWithJoysticks(drivetrain, m_JoystickLeft, m_JoystickRight);

    // Gripper
    gripperOpen = new cm_GripperOpen(gripper);
    gripperClose = new cm_GripperClose(gripper);

    // Put the drive train sendable values onto the networktables / dashboard
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(gripper);

    // Auto Bench-Test

    autoSquare = new ca_driveAutoSquare(drivetrain, TrajectoryContainer.trajectoryf);

    // Going Backword-mobility.

    autoTrajectory = new ca_autoTrajectory(drivetrain, TrajectoryContainer.pigeontraj);

    autoTrajectoryKinematic = new ca_autoTrajectoryKinematic(drivetrain, TrajectoryContainer.pigeontraj);

    // Be able to turn

    autoTurnTrajectory = new ca_autoTurnKinematic(drivetrain, 0.0, -135.0); // testing 90 degree Turn;

    // Make a point & go Backword-mobility.

    autoScore = new cg_autoScore(drivetrain, arm, gripper);

    // Make 2 point and go Backword-mobility.

    autoDoubleScore = new cg_autoDoubleScore(drivetrain, arm, gripper);

    // Go backword and do autobalancing.

    // Go in a simple L shape and do auto balancing.

    // Make 2 points and go in a simple L shape and do auto balancing.

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

    // Arduino Controller Button Mapping
    // Arm Movement
    new JoystickButton(m_ArduinoController, Constants.groundButtonID).whileTrue(setArmPositionLow);
    new JoystickButton(m_ArduinoController, Constants.middleButtonID).whileTrue(setArmPositionMiddle);
    new JoystickButton(m_ArduinoController, Constants.highButtonID).whileTrue(setArmPositionHigh);
// manual extension
    new JoystickButton(m_ArduinoController, Constants.ardJoystickUp).onTrue(increaseExtensionTicks);
    new JoystickButton(m_ArduinoController, Constants.ardJoystickDown).onTrue(decreaseExtensionTicks);
//reset manual extension
    new JoystickButton(m_ArduinoController, Constants.highButtonID).onFalse(resetExtensionTicks);
    new JoystickButton(m_ArduinoController, Constants.middleButtonID).onFalse(resetExtensionTicks);
    new JoystickButton(m_ArduinoController, Constants.groundButtonID).onFalse(resetExtensionTicks);
    // Toggle Switches
    new JoystickButton(m_ArduinoController, Constants.gamePieceID).onTrue(setGamePieceTypeCubeTrue)
        .onFalse(setGamePieceTypeCubeFalse);

    new JoystickButton(m_ArduinoController, Constants.robotSideID).whileTrue(setSideOrientationBattery)
        .whileFalse(setSideOrientationCompressor);

    new JoystickButton(m_ArduinoController, Constants.gripperID).whileTrue(gripperOpen).whileFalse(gripperClose);

    // __________________________

    //new JoystickButton(m_ArduinoController, Constants.ardJoystickUp).whileTrue(moveArmCompressor);
    //new JoystickButton(m_ArduinoController, Constants.ardJoystickDown).whileTrue(moveArmBattery);
    //new JoystickButton(m_ArduinoController, Constants.ardJoystickLeft).whileTrue(moveArmLeft);
    //new JoystickButton(m_ArduinoController, Constants.ardJoystickRight).whileTrue(moveArmRight);

    // __________________________

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoTrajectory;

    // A command will be run in autonomous
    // return forwardHalfSpeed;
  }

}
