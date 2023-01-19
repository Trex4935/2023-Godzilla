// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Gyro Imports
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickAxis;
import frc.robot.Constants.MovementConstraints;
import frc.robot.Constants.WheelIDConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.extensions.Talon;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase {

    // Declaring Motors
    WPI_TalonSRX FLMotor;
    WPI_TalonSRX FRMotor;
    WPI_TalonSRX MLMotor;
    WPI_TalonSRX MRMotor;
    WPI_TalonSRX BLMotor;
    WPI_TalonSRX BRMotor;

    // Declaring Motor Groups
    MotorControllerGroup leftMotors;
    MotorControllerGroup rightMotors;

    DifferentialDrive diffdrive;

    // Declaring encoders
    Encoder leftEncoder;
    Encoder rightEncoder;
    
    XboxController xboxController;

    // Declaring Gyro Objects
    public static AHRS ahrs;

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

        leftEncoder = new Encoder(4, 5);
        rightEncoder = new Encoder(6, 7);

        // in Inches
        leftEncoder.setDistancePerPulse((DrivetrainConstants.wheelDiameter * Math.PI) / DrivetrainConstants.encoderTicks);
        rightEncoder.setDistancePerPulse((DrivetrainConstants.wheelDiameter * Math.PI) / DrivetrainConstants.encoderTicks);

        // Creating gyro object
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    /** Resets the gyro */
    public void resetGyro() {
        ahrs.reset();
    }

    /** Gets Roll(X) angle from Gyro */
    public Float getXAngle() {
        return ahrs.getRoll();
    }
    
    /**  Gets Pitch(Y) angle from Gyro */
    public Float getYAngle() {
        return ahrs.getPitch();
    }

    /** Gets Yaw(Z) angle from Gyro */
    public Float getZAngle() {
        return ahrs.getYaw();
    }

    /** Creates an array of the roll, pitch, and yaw values */
    public float[] PrincipalAxisValues() {
        return new float[] {getXAngle(), getYAngle(), getZAngle()};
    }

    public void HalfSpeed(){
        leftMotors.set(0.5);
        rightMotors.set(0.5);
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

    /** Sets the max speed value (sendable) */ 
    public void setMaxSpeed(double MaxSpeed) {
        m_MaxSpeed = MaxSpeed;
    }

    /** Get the Max speed value (sendable) */ 
    public double getMaxSpeed() {
        return m_MaxSpeed;
    }

    /** Resets both encoders to 0 */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /** Gets the amount of ticks since reset/init based on setDistancePerPulse */
    public double getLeftEncoderTicks() {
        return leftEncoder.getDistance();
    }

    /** Gets the amount of ticks since reset/init based on setDistancePerPulse */
    public double getRightEncoderTicks() {
        return rightEncoder.getDistance();
    }

    /** Gets the speed based on setDistancePerPulse */
    public double getLeftEncoderSpeed() {
        return leftEncoder.getRate();
    }

    /** Gets the speed based on setDistancePerPulse */
    public double getRightEncoderSpeed() {
        return rightEncoder.getRate();
    }

    /** Converts inches to ticks for motors */
    public double inchesToTicks(double inches) {
        // 6 diameter wheel
        // 2048 ticks per motor rotation.
        // Gear ratio - 44:30
            // Driver and driven unknown
        // 18.85 inches of travel per rotation of the wheel   
        // 1.47 motor rotations = 18.85 inches of travel
        // 3011 ticks per wheel rotation
        // 3011 ticks per 18.85 inches
        // 160 ticks per 1 inch of travel
        
        return inches * 160;
    }

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MaxSpeed", this::getMaxSpeed, this::setMaxSpeed);
        builder.addFloatArrayProperty("Roll, Pitch, and Yaw Values", this::PrincipalAxisValues, null);
    }
}
