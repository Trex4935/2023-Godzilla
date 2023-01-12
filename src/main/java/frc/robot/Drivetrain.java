// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxisIDConstants;
import frc.robot.Constants.MovementConstraints;
import frc.robot.Constants.WheelIDConstants;
import frc.robot.extensions.Talon;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase {

/**
 * F = Front  | L = Left
 * M = Middle | R = Right
 * B = Back   | 
 */
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
    
    // Creates differential drive configuration with groups.
    diffdrive = new DifferentialDrive(leftMotors, rightMotors);

    // Sets max speed of drivetrain
    diffdrive.setMaxOutput(MovementConstraints.dtmaxspeed);

}

/** Sets speed to the axis values of Xbox Controller*/
public void driveWithController(XboxController xboxController) {
    diffdrive.tankDrive(xboxController.getRawAxis(AxisIDConstants.leftIDAxis), 
    xboxController.getRawAxis(AxisIDConstants.rightIDAxis));
}

/** Stops motors */
public void stopMotors() {
    leftMotors.set(0);
    rightMotors.set(0);
}

}
