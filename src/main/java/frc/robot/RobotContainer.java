// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Arm;
import frc.robot.commands.autoGroups.cg_unifiedAuto;
import frc.robot.commands.teleop.ca_ArmMovementCombo;
import frc.robot.commands.teleop.ca_setSideOrientation;
import frc.robot.commands.teleop.cm_GripperClose;
import frc.robot.commands.teleop.cm_GripperOpen;
import frc.robot.commands.teleop.cm_driveWithJoysticks;
import frc.robot.commands.teleop.cm_manualAddExtendTicks;
import frc.robot.commands.teleop.cm_manualDecreaseExtendTicks;
import frc.robot.commands.teleop.cm_manualResetAddArm;
import frc.robot.commands.teleop.cm_manualRotateBattery;
import frc.robot.commands.teleop.cm_manualRotateCompressor;
import frc.robot.commands.teleop.cm_setArmPositionManual;
import frc.robot.commands.teleop.cm_setGamePieceType;
import frc.robot.commands.teleop.cm_setSpeedLimit;
// Misc
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.extensions.DriveState;

// Robot Base Class
public class RobotContainer {

  // Declare Subsystems
  public final Drivetrain drivetrain;
  public final Arm arm;
  private final Gripper gripper;

  // Declare Commands
  public final cm_driveWithJoysticks driveWithJoysticks;
  private final cm_setSpeedLimit setSpeedLimitMax;
  public final ca_ArmMovementCombo armMovementCombo;
  private final cm_setArmPositionManual setArmPositionHigh;
  private final cm_setArmPositionManual setArmPositionMiddle;
  private final cm_setArmPositionManual setArmPositionLow;
  private final cm_setArmPositionManual setArmPositionShelf;
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
  private final cg_unifiedAuto unifiedAuto;
  
  // Declare Other
  private final Joystick m_JoystickRight = new Joystick(Constants.rightJoystick);
  private final Joystick m_JoystickLeft = new Joystick(Constants.leftJoystick);
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

    // Autonomous
    unifiedAuto = new cg_unifiedAuto(arm, gripper, drivetrain);

    // Arm
    armMovementCombo = new ca_ArmMovementCombo(arm);

    // Drivetrain
    driveWithJoysticks = new cm_driveWithJoysticks(drivetrain, m_JoystickRight, m_JoystickLeft);
    setSpeedLimitMax = new cm_setSpeedLimit(DriveState.TURBO);
    
    /// Operator ///

    // Manual control of arm
    manualResetAddArm = new cm_manualResetAddArm(arm);
    manualAddExtendTicks = new cm_manualAddExtendTicks(arm);
    manualDecreaseExtendTicks = new cm_manualDecreaseExtendTicks(arm);
    manualRotateCompressor = new cm_manualRotateCompressor(arm);
    manualRotateBattery = new cm_manualRotateBattery(arm);

    // Set arm positions
    setArmPositionHigh = new cm_setArmPositionManual(ArmPosition.HIGH);
    setArmPositionMiddle = new cm_setArmPositionManual(ArmPosition.MIDDLE);
    setArmPositionLow = new cm_setArmPositionManual(ArmPosition.LOW);
    setArmPositionShelf = new cm_setArmPositionManual(ArmPosition.SHELF);

    // Setting arm orientation
    setSideOrientationCompressor = new ca_setSideOrientation(ArmSideOrientation.CompressorSide);
    setSideOrientationBattery = new ca_setSideOrientation(ArmSideOrientation.BatterySide);

    // Setting game piece type
    setGamePieceTypeCubeTrue = new cm_setGamePieceType("cube");
    setGamePieceTypeCubeFalse = new cm_setGamePieceType("cone");

    // Gripper
    gripperOpen = new cm_GripperOpen(gripper);
    gripperClose = new cm_GripperClose(gripper);

    // Puts the subsystem sendibles on the dashboard
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(gripper);

    // Configure control bindings
    configureBindings();

  }

  /** Use to define trigger->command mappings. */
  private void configureBindings() {

    // Makes controller driving the default command
    // drivetrain.setDefaultCommand(driveWithJoysticks);

    // Runs the arm state machine
    // arm.setDefaultCommand(armMovementCombo);

    // Increase Speed when pressing triggers.
    new JoystickButton(m_JoystickRight, Constants.joystickTrigger).whileTrue(setSpeedLimitMax);
    // new JoystickButton(m_JoystickRight, 3).onTrue(changeDirection);

    // Arduino Controller Button Mapping
    // Arm Movement
    new JoystickButton(m_ArduinoController, Constants.groundButtonID).whileTrue(setArmPositionLow)
        .onFalse(manualResetAddArm);
    new JoystickButton(m_ArduinoController, Constants.middleButtonID).whileTrue(setArmPositionMiddle)
        .onFalse(manualResetAddArm);
    new JoystickButton(m_ArduinoController, Constants.highButtonID).whileTrue(setArmPositionHigh)
        .onFalse(manualResetAddArm);
    new JoystickButton(m_ArduinoController, Constants.shelfButtonID).whileTrue(setArmPositionShelf)
        .onFalse(manualResetAddArm);

    // manual EXTENSION
    new JoystickButton(m_ArduinoController, Constants.ardJoystickUp).whileTrue(manualAddExtendTicks);
    new JoystickButton(m_ArduinoController, Constants.ardJoystickDown).whileTrue(manualDecreaseExtendTicks);
    // manual ROTATION
    new JoystickButton(m_ArduinoController, Constants.ardJoystickLeft).whileTrue(manualRotateCompressor);
    new JoystickButton(m_ArduinoController, Constants.ardJoystickRight).whileTrue(manualRotateBattery);

    // Toggle Switches
    new JoystickButton(m_ArduinoController, Constants.gamePieceID).onTrue(setGamePieceTypeCubeTrue)
        .onFalse(setGamePieceTypeCubeFalse);

    new JoystickButton(m_ArduinoController, Constants.robotSideID).onTrue(setSideOrientationCompressor)
        .onFalse(setSideOrientationBattery);

    new JoystickButton(m_ArduinoController, Constants.gripperID).whileTrue(gripperOpen).whileFalse(gripperClose);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // return autoBalance.withTimeout(15);
    return unifiedAuto;
    // return mobilityUnifiedAuto;

  }
}
