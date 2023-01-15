// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickAxis;
import frc.robot.Constants.MovementConstraints;
import frc.robot.Constants.WheelIDConstants;
import frc.robot.extensions.Talon;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase {

    WPI_TalonSRX FLMotor;
    WPI_TalonSRX FRMotor;
    WPI_TalonSRX MLMotor;
    WPI_TalonSRX MRMotor;
    WPI_TalonSRX BLMotor;
    WPI_TalonSRX BRMotor;

    MotorControllerGroup leftMotors;
    MotorControllerGroup rightMotors;

    DifferentialDrive diffdrive;

    XboxController xboxController;

    // Max speed value for the motors ... default comes from constants
    Double m_MaxSpeed = MovementConstraints.dtmaxspeed;

    public Drivetrain() {
        // Creates new motor objects and configures the talons in a separate method
        FLMotor = Talon.createDefaultTalon(WheelIDConstants.FLMotorID);
        FRMotor = Talon.createDefaultTalon(WheelIDConstants.FRMotorID);
        MLMotor = Talon.createDefaultTalon(WheelIDConstants.MLMotorID);
        MRMotor = Talon.createDefaultTalon(WheelIDConstants.MRMotorID);
        BLMotor = Talon.createDefaultTalon(WheelIDConstants.BLMotorID);
        BRMotor = Talon.createDefaultTalon(WheelIDConstants.BRMotorID);
        // Sets up motor controller groups
        leftMotors = new MotorControllerGroup(FLMotor, MLMotor, BLMotor);
        rightMotors = new MotorControllerGroup(FRMotor, MRMotor, BRMotor);

        diffdrive = new DifferentialDrive(leftMotors, rightMotors);

        diffdrive.setMaxOutput(m_MaxSpeed);

    }

    public void driveWithController(double leftSpeed, double rightSpeed) {
        diffdrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveWithJoysticks(Joystick joystick1, Joystick joystick2) {
        diffdrive.tankDrive(joystick1.getRawAxis(JoystickAxis.joystickAxis),
                joystick2.getRawAxis(JoystickAxis.joystickAxis));
    }

    public void stopMotors() {
        leftMotors.set(0);
        rightMotors.set(0);
    }

    // Sets the max speed value (sendable)
    public void setMaxSpeed(double MaxSpeed) {
        m_MaxSpeed = MaxSpeed;
    }

    // Get the Max speed value (sendable)
    public double getMaxSpeed() {
        return m_MaxSpeed;
    }

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MaxSpeed", this::getMaxSpeed, this::setMaxSpeed);
    }
}
