// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Gyro Imports
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extensions.Helper;
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

    // Declare diff drive
    DifferentialDrive diffdrive;

    // Declaring encoders
    private static Encoder leftEncoder;
    private static Encoder rightEncoder;

    // Declaring Gyro Objects
    public static AHRS ahrs;

    // Max speed value for the motors ... default comes from constants
    Double m_MaxSpeed = Constants.dtmaxspeed;

    // Drivetrain contructor
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

        // Create differential drive object
        diffdrive = new DifferentialDrive(leftMotors, rightMotors);
        diffdrive.setMaxOutput(m_MaxSpeed);

        // Create encoders and set distance values
        leftEncoder = new Encoder(1, 2);
        rightEncoder = new Encoder(3, 4);
        // 160 ticks per inch ... so get distance should get inches traveled
        leftEncoder.setDistancePerPulse(.00975);
        rightEncoder.setDistancePerPulse(.00975);

        // Creating gyro object
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.calibrate();
    }

    // DEFAULT Command that moves the robot with joysticks
    public void driveWithJoysticks(Joystick leftJoystick, Joystick rightJoystick) {
        diffdrive.tankDrive(-leftJoystick.getRawAxis(Constants.joystickAxis),
                -rightJoystick.getRawAxis(Constants.joystickAxis));
    }

    /** Resets the gyro */
    public void resetGyro() {
        ahrs.reset();
    }

    /** Gets Yaw(Z) angle from Gyro and converts it to 360 */
    public double getZAngleConverted() {
        double rollBoundedDouble = s_getAngleZ().doubleValue();
        return Helper.ConvertTo360(rollBoundedDouble);
    }

    // Uses gyro to go to position w/ drive straight
    public boolean driveStraightTarget(double Speed, double Angle, double Position) {
        // Beep boop thingy mabober-computer- tels us what its doing
        // stops when reaches position
        if (reachDriveTarget(Position)) {
            stopMotors();
            return true;
        } else {
            double err = Angle + getZAngleConverted();
            double P = 0.001;
            double driftCorrection = err * P;
            diffdrive.arcadeDrive(Speed, driftCorrection);
            return false;
        }
    }

    /** Stops all Drivetrain motor groups. */
    public void stopMotors() {
        leftMotors.stopMotor();
        rightMotors.stopMotor();
    }

    /** Sets the max speed value (sendable) */
    public void setMaxSpeed(double MaxSpeed) {
        m_MaxSpeed = MaxSpeed;
        diffdrive.setMaxOutput(m_MaxSpeed);
    }

    /** Resets both encoders to 0 */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
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

    // Uses gyro to go to position w/ drive straight
    public boolean turnToTarget(double angle) {
        if (reachTurnAngle(angle)) {
            stopMotors();
            return true;
        } else {
            double err = angle + getZAngleConverted();
            double P = 0.001;
            double driftCorrection = err * P;
            diffdrive.arcadeDrive(0, driftCorrection);
            return false;
        }
    }

    // Chages encoder ticks to meters to make the constant easier to input for
    // position.
    public Boolean reachDriveTarget(Double targetPosition) {
        double averageTickValue = (Math.abs(leftEncoder.getDistance()) + Math.abs(rightEncoder.getDistance()))
                * (Math.signum(targetPosition)) / 2;
        // System.out.println(" targetPosition: " + targetPosition + " averageDistance:
        // " + averageTickValue);

        // if tick value is greater than or equal to target position, stop both motors
        if (averageTickValue >= targetPosition - 0.5) {
            return true;
        } else {
            return false;
        }
    }

    public Boolean reachTurnAngle(Double targetAngle) {
        if (Helper.RangeCompare(targetAngle + 1, targetAngle - 1, getZAngleConverted())) {
            return true;
        } else {
            return false;
        }
    }

    // ******************** Sendables ********************

    /** Gets Roll(X) angle from Gyro */
    public Float s_getAngleX() {
        return ahrs.getRoll();
    }

    /** Gets Pitch(Y) angle from Gyro */
    public Float s_getAngleY() {
        // return ahrs.getYaw();
        return ahrs.getPitch();
    }

    /** Gets Yaw(Z) from Gyro */
    public Float s_getAngleZ() {
        // return ahrs.getRoll();
        return ahrs.getYaw();
    }

    /** Get the Max speed value (sendable) */
    public double s_getMaxSpeed() {
        return m_MaxSpeed;
    }

    /** Gets the distance since reset/init based on setDistancePerPulse */
    public double s_getEncoderLeftDistance() {
        return leftEncoder.getDistance();
    }

    /** Gets the speed based on setDistancePerPulse */
    public double s_getEncoderLeftSpeed() {
        return leftEncoder.getRate();
    }

    /** Gets the amount of ticks since reset/init */
    public double s_getEncoderLeftTicks() {
        return leftEncoder.get();
    }

    /** Gets the distance since reset/init based on setDistancePerPulse */
    public double s_getEncoderRightDistance() {
        return rightEncoder.getDistance();
    }

    /** Gets the speed based on setDistancePerPulse */
    public double s_getEncoderRightSpeed() {
        return rightEncoder.getRate();
    }

    /** Gets the amount of ticks since reset/init */
    public double s_getEncoderRightTicks() {
        return -rightEncoder.get();
    }

    public boolean s_getDoAutoBalance() {
        return Constants.doAutoBalance;
    }

    public void s_setDoAutoBalance(boolean autoStatus) {
        Constants.doAutoBalance = autoStatus;
    }

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MaxSpeed", this::s_getMaxSpeed, this::setMaxSpeed);
        // Gyro values
        builder.addFloatProperty("Roll", this::s_getAngleX, null);
        builder.addFloatProperty("Pitch", this::s_getAngleY, null);
        builder.addFloatProperty("Yaw", this::s_getAngleZ, null);
        builder.addDoubleProperty("RightEncoder", this::s_getEncoderRightTicks, null);
        builder.addDoubleProperty("LeftEncoder", this::s_getEncoderLeftTicks, null);
        builder.addDoubleProperty("Left Encoder Speed", this::s_getEncoderLeftSpeed, null);
        builder.addDoubleProperty("Right Encoder Speed", this::s_getEncoderRightSpeed, null);
        builder.addDoubleProperty("Left Encoder Distance", this::s_getEncoderLeftDistance, null);
        builder.addDoubleProperty("Right Encoder Distance", this::s_getEncoderRightDistance, null);
        builder.addBooleanProperty("Auto Balance?", this::s_getDoAutoBalance, this::s_setDoAutoBalance);
    }
}
