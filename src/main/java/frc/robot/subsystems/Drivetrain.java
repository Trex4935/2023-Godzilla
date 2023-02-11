// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Gyro Imports
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.extensions.Helper;
import frc.robot.extensions.PID;

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


    // PID
    PIDController drivePID;

    // Declaring Gyro Objects
    public static AHRS ahrs;

    // Max speed value for the motors ... default comes from constants
    Double m_MaxSpeed = Constants.dtmaxspeed;

    // Kinemtatics
    DifferentialDriveKinematics kin;

    // Simulate
    public double zSimAngle;

    public Drivetrain() {

        // Creates new motor objects and configures the talons in a separate method
        FLMotor = Talon.createDefaultTalon(Constants.FLMotorID);
        FRMotor = Talon.createDefaultTalon(Constants.FRMotorID);
        MLMotor = Talon.createDefaultTalon(Constants.MLMotorID);
        MRMotor = Talon.createDefaultTalon(Constants.MRMotorID);
        BLMotor = Talon.createDefaultTalon(Constants.BLMotorID);
        BRMotor = Talon.createDefaultTalon(Constants.BRMotorID);
        // Sets up motor controller groups
        leftMotors = new MotorControllerGroup(FLMotor, MLMotor, BLMotor);
        rightMotors = new MotorControllerGroup(FRMotor, MRMotor, BRMotor);
        // Inverts direction of motors/wheels.
        leftMotors.setInverted(true);
        rightMotors.setInverted(false);
        FLMotor.setNeutralMode(NeutralMode.Brake);
        FRMotor.setNeutralMode(NeutralMode.Brake);
        MLMotor.setNeutralMode(NeutralMode.Brake);
        MRMotor.setNeutralMode(NeutralMode.Brake);
        BLMotor.setNeutralMode(NeutralMode.Brake);
        BRMotor.setNeutralMode(NeutralMode.Brake);

        diffdrive = new DifferentialDrive(leftMotors, rightMotors);

        diffdrive.setMaxOutput(m_MaxSpeed);

        leftEncoder = new Encoder(1, 2);
        rightEncoder = new Encoder(3, 4);

        // in Inches
        leftEncoder.setDistancePerPulse((Constants.wheelDiameter * Math.PI) / Constants.encoderTicks);
        rightEncoder.setDistancePerPulse((Constants.wheelDiameter * Math.PI) / Constants.encoderTicks);

        // Creating gyro object
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.calibrate();

        // Distance between 2 wheel godzilla 641 mm, to do find or measure same for mrT
        kin = new DifferentialDriveKinematics(TrajectoryConstants.kTrackWidthMeters);

        // initiate simulate gyro Position
        zSimAngle = 0;

        drivePID = new PIDController(0.03, 0.0, 0);

    }

    /** Resets the gyro */
    public void resetGyro() {
        ahrs.reset();
    }

    /** Gets Roll(X) angle from Gyro */
    public Float getXAngle() {
        return ahrs.getRoll();
    }

    /** Gets Pitch(Y) angle from Gyro */
    public Float getYAngle() {
        return ahrs.getPitch();
    }


    /** Gets Yaw(Z) angle from Gyro */
    public Float getZAngle() {
        return -ahrs.getYaw();

    }


    /** Gets Yaw(Z) angle from Gyro */
    public double getZAngleConverted() {
        Float yawBounded = -ahrs.getYaw();
        double yawBoundedDouble = yawBounded.doubleValue();
        return Helper.ConvertTo360(yawBoundedDouble);
    }

    /** Creates an array of the roll, pitch, and yaw values */
    public float[] PrincipalAxisValues() {
        return new float[] { getXAngle(), getYAngle(), getZAngle() };
    }

    public void HalfSpeed() {
        leftMotors.set(0.5);
        rightMotors.set(0.5);
    }

    public void driveWithController(double leftSpeed, double rightSpeed) {
        diffdrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveWithJoysticks(Joystick joystick1, Joystick joystick2) {
        diffdrive.tankDrive(joystick1.getRawAxis(Constants.joystickAxis),
                joystick2.getRawAxis(Constants.joystickAxis));
    }

    /** Stops all Drivetrain motor groups. */
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

    /** Move the robot based on its pitch/y axis */
    public void autoBalance() {
        // Set motors speed using PID controller to get Y-axis to 0 degrees
        leftMotors.set(drivePID.calculate(getYAngle(), 0));
        rightMotors.set(drivePID.calculate(getYAngle(), 0));
    }

    /** Gets the amount of ticks since reset/init */
    public double getLeftEncoderTicks() {
        return leftEncoder.get();
    }

    /** Gets the amount of ticks since reset/init */
    public double getRightEncoderTicks() {
        return rightEncoder.get();
    }

    /** Gets the distance since reset/init based on setDistancePerPulse */
    public double getLeftEncoderDistance() {
        return leftEncoder.getDistance();
    }

    /** Gets the distance since reset/init based on setDistancePerPulse */
    public double getRightEncoderDistance() {
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

    // takes in chasis speed and chasis angular rate or rotation and return the left
    // speed of the wheel;
    public double getLeftSpeedKin(double chassisSpeedx, double chassisAngularRate) {
        double chassisSpeedy = 0;
        DifferentialDriveWheelSpeeds wheelSpeed = kin
                .toWheelSpeeds(new ChassisSpeeds(chassisSpeedx, chassisSpeedy, chassisAngularRate));
        return wheelSpeed.leftMetersPerSecond;
    }

    // takes in chasis speed and chasis angular rate or rotation and return the
    // right speed of the wheel;
    public double getRightpeedKin(double chassisSpeedx, double chassisAngularRate) {
        double chassisSpeedy = 0;
        DifferentialDriveWheelSpeeds wheelSpeed = kin
                .toWheelSpeeds(new ChassisSpeeds(chassisSpeedx, chassisSpeedy, chassisAngularRate));
        ;
        return wheelSpeed.rightMetersPerSecond;
    }

    public void simulateGyro(double leftSpeed, double rightSpeed, Timer timer) {

        ChassisSpeeds chassisSpeed = kin.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
        zSimAngle = chassisSpeed.omegaRadiansPerSecond * 0.02 + zSimAngle;
    }

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MaxSpeed", this::getMaxSpeed, this::setMaxSpeed);
        builder.addFloatArrayProperty("Roll, Pitch, and Yaw Values", this::PrincipalAxisValues, null);
        builder.addDoubleProperty("RightEncoder", this::getRightEncoderTicks, null);
        builder.addDoubleProperty("LeftEncoder", this::getLeftEncoderTicks, null);
    }
}
