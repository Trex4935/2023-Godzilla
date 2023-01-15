// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.cm_ExtendArm;
import frc.robot.commands.cm_driveWithJoysticks;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain dt;
  private final ArmExtension arm;
  private final cm_driveWithJoysticks driveWtithJoysticks;
  private final cm_ExtendArm extendArm;

  private final Joystick m_JoystickLeft = new Joystick(0);
  private final Joystick m_JoystickRight = new Joystick(1);

  private final XboxController operator = new XboxController(2);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    dt = new Drivetrain();
    arm = new ArmExtension();

    driveWtithJoysticks = new cm_driveWithJoysticks(dt,m_JoystickLeft, m_JoystickRight);
    extendArm = new cm_ExtendArm(arm, operator);

    // Configure the trigger bindings
    configureBindings();
  }

  /** Use to define trigger->command mappings.*/
  private void configureBindings() {
    
    // Makes controller driving the default command
    dt.setDefaultCommand(driveWtithJoysticks);
    arm.setDefaultCommand(extendArm);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command will be run in autonomous
    return null;
  }


}
