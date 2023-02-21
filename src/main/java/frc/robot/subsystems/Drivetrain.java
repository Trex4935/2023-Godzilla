// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Gyro Imports
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoMovementConstraints;
import frc.robot.Constants.TrajectoryConstants;
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

    DifferentialDrive diffdrive;

    double trajPos;

    double trajSpeed;

    // PID
    PIDController drivePID;

    // Declaring Gyro Objects
    public static AHRS ahrs;

    // Max speed value for the motors ... default comes from constants
    Double m_MaxSpeed = Constants.dtmaxspeed;

    // Kinemtatics
    DifferentialDriveKinematics kin;

    //
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.105 * 12, 0.72);

    private final Encoder m_leftEncoder = new Encoder(1, 2);
    private final Encoder m_rightEncoder = new Encoder(3, 4);

    private final PIDController m_leftPIDController = new PIDController(0, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(0, 0, 0);

    private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(leftMotors);
    private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(rightMotors);

    //

    // Simulate
    public double zSimAngle;

    public Drivetrain() {

        trajPos = 0;
        trajSpeed = 0;
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

        // in Inches
        m_leftEncoder.setDistancePerPulse(Units.inchesToMeters(139.565 / 14171));
        m_rightEncoder.setDistancePerPulse(Units.inchesToMeters(139.565 / 14171)); // 0.00230097
        // Constants.wheelDiameter * Math.PI) / Constants.encoderTicks

        // Creating gyro object
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.calibrate();
        ahrs.reset();

        // Distance between 2 wheel godzilla 641 mm, to do find or measure same for mrT
        kin = new DifferentialDriveKinematics(TrajectoryConstants.kTrackWidthMeters);

        // initiate simulate gyro Position
        zSimAngle = 0;

        drivePID = new PIDController(0.15, 0.0, 0);

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
    //    return ahrs.getYaw();
        return ahrs.getPitch();
    }

    
    /** Gets the offset of the pitch*/
    public Float getYAngleOffset() {
        //    return ahrs.getYaw();
            return getYAngle()  
            - 1.75f;
        }
    

    /** Gets Yaw(Z) from Gyro */
    public Float getZAngle() {
    //   return ahrs.getRoll();
       return -ahrs.getYaw();

    }

    /** Gets Yaw(Z) angle from Gyro and converts it to 360 */
    public double getZAngleConverted() {
        double rollBoundedDouble = getZAngle().doubleValue();
        return Helper.ConvertTo360(rollBoundedDouble);
    }

    /** Creates an array of the yaw, pitch, and roll values */
    public float[] PrincipalAxisValues() {
        return new float[] { getXAngle(), getYAngle(), getZAngle() };
    }

    public void HalfSpeed() {
        leftMotors.set(0.105);
        rightMotors.set(0.105);
    }

    public void driveWithController(double leftSpeed, double rightSpeed) {
        diffdrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveWithStraightWithGyro(double avgSpeed, double targetAngle) {
        double err = targetAngle - getZAngleConverted();
        double P = 0.001;
        double driftCorrection = err * P;
        diffdrive.arcadeDrive(avgSpeed, driftCorrection);
    }

    public void driveWithJoysticks(Joystick joystick1, Joystick joystick2) {
        diffdrive.tankDrive(-joystick1.getRawAxis(Constants.joystickAxis),
                -joystick2.getRawAxis(Constants.joystickAxis));
    }

    /** Stops all Drivetrain motor groups. */
    public void stopMotors() {
        leftMotors.set(0);
        rightMotors.set(0);
    }

    /** Sets the max speed value (sendable) */
    public void setMaxSpeed(double MaxSpeed) {
        m_MaxSpeed = MaxSpeed;
        diffdrive.setMaxOutput(m_MaxSpeed);
    }

    /** Get the Max speed value (sendable) */
    public double getMaxSpeed() {
        return m_MaxSpeed;
    }

    /** Resets both encoders to 0 */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /** Move the robot based on its pitch/y axis */
    public void autoBalance() {
        // Set motors speed using PID controller to get Y-axis to 0 degrees
        double leftPitch = drivePID.calculate(getYAngleOffset(), 0);
        double rightPitch = drivePID.calculate(getYAngleOffset(), 0);        
        double err = 0 - getZAngleConverted();
        double P = 0.01;
        double driftCorrectionTwist = err * P;

        leftMotors.set(leftPitch +  driftCorrectionTwist);
        rightMotors.set(rightPitch - driftCorrectionTwist);
    }

    /** Gets the amount of ticks since reset/init */
    public double getLeftEncoderTicks() {
        return m_leftEncoder.get();
    }

    /** Gets the amount of ticks since reset/init */
    public double getRightEncoderTicks() {
        return -m_rightEncoder.get();
    }

    /** Gets the distance since reset/init based on setDistancePerPulse */
    public double getLeftEncoderDistance() {
        return m_leftEncoder.getDistance();
    }

    /** Gets the distance since reset/init based on setDistancePerPulse */
    public double getRightEncoderDistance() {
        return m_rightEncoder.getDistance();
    }

    /** Gets the speed based on setDistancePerPulse */
    public double getLeftEncoderSpeed() {
        return m_leftEncoder.getRate();
    }

    /** Gets the speed based on setDistancePerPulse */
    public double getRightEncoderSpeed() {
        return m_rightEncoder.getRate();
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

    public double getOmega(double startAngle, double endAngle) {
        double omega = 0;
        if (startAngle > endAngle) { // if Start > End , go left, w +
            omega = AutoMovementConstraints.dtmaxomega;
        } else { // if Start < End, go right, w -
            omega = -AutoMovementConstraints.dtmaxomega;
        }

        return omega;
    }

    public void setTrajPos(State currState) {
        trajPos = currState.poseMeters.getY();
    }

    public void setTrajSpeed(State currState) {
        trajSpeed = currState.velocityMetersPerSecond;
    }

    public Double getTrajPos() {
        return trajPos;
    }

    public Double getTrajSpeed() {
        return trajSpeed;
    }

    public void driveTankWithStateTraj(State currState, Double end, Double time) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        driveWithController(velocityTarget * Math.signum(end), velocityTarget * Math.signum(end));
        // System.out.println("Time:"+ time + "Velocity:" + velocityTarget + "Position:"
        // + currState.poseMeters.getY());
    }

    public void driveTankWithStateKinematicTraj(State currState, Double end, Double time) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        // Rate is 0, because we are following a straight line, the speed varies
        // depending of path, it follows a trapezoid curve.
        Double leftSpeedWheel = getLeftSpeedKin(velocityTarget, 0);
        Double rightSpeedWheel = getRightpeedKin(velocityTarget, 0);
        driveWithController(leftSpeedWheel * Math.signum(end), rightSpeedWheel * Math.signum(end));
        // System.out.println("Time: "+ time + " Velocity: " + velocityTarget + "
        // Position: " + currState.poseMeters.getY() + " LeftSpeed: " + leftSpeedWheel +
        // " RightSpeed: " + rightSpeedWheel);
    }

    public void driveArcadeWithStateKinematicGyroTraj(State currState, Double end, Double time, Double targetAngle) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        // Rate is 0, because we are following a straight line, the speed varies
        // depending of path, it follows a trapezoid curve.
        Double leftSpeedWheel = getLeftSpeedKin(velocityTarget, 0);
        Double rightSpeedWheel = getRightpeedKin(velocityTarget, 0);
        driveWithStraightWithGyro(velocityTarget * Math.signum(end), targetAngle);
        // System.out.println("Time: "+ time + " Velocity: " + velocityTarget + "
        // Position: " + currState.poseMeters.getY() + " LeftSpeed: " + leftSpeedWheel +
        // " RightSpeed: " + rightSpeedWheel);

    }

    public void driveWithPIDTank(State currState, Double end, Double time) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        // Rate is 0, because we are following a straight line, the speed varies
        // depending of path, it follows a trapezoide curve.
        Double leftSpeedWheel = getLeftSpeedKin(velocityTarget, 0);
        Double rightSpeedWheel = getRightpeedKin(velocityTarget, 0);
        setSpeeds(leftSpeedWheel, rightSpeedWheel);
        // System.out.println("Time: "+ time + " Velocity: " + velocityTarget + "
        // Position: " + currState.poseMeters.getY() + " LeftSpeed: " + leftSpeedWheel +
        // " RightSpeed: " + rightSpeedWheel);

    }

    public void driveWithPIDArcade(State currState, Double end, Double time, Double angle) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        // Rate is 0, because we are following a straight line, the speed varies
        // depending of path, it follows a trapezoide curve.
        Double leftSpeedWheel = getLeftSpeedKin(velocityTarget, 0);
        Double rightSpeedWheel = getRightpeedKin(velocityTarget, 0);
        // TO DO
        double err = angle - getZAngleConverted();
        double P = 0.1;
        double driftCorrectionTwist = err * P;
        Double leftSpeedWheelWithGyroCorrection = leftSpeedWheel - driftCorrectionTwist;
        Double rightSpeedWheelWithGyroCorrection = rightSpeedWheel + driftCorrectionTwist;
        //
        setSpeeds(leftSpeedWheelWithGyroCorrection, rightSpeedWheelWithGyroCorrection);
        // System.out.println("Time: "+ time + " Velocity: " + velocityTarget + "
        // Position: " + currState.poseMeters.getY() + " LeftSpeed: " + leftSpeedWheel +
        // " RightSpeed: " + rightSpeedWheel);

    }

    public void setSpeeds(Double leftSpeedWheel, Double rightSpeedWheel) {
        final double leftFeedforward = m_feedforward.calculate(leftSpeedWheel);
        final double rightFeedforward = m_feedforward.calculate(-rightSpeedWheel);

        final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), leftSpeedWheel); // is encoder
                                                                                                          // in ticks
                                                                                                          // per/sec or
                                                                                                          // m/sec
        final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), rightSpeedWheel);
        FLMotor.setVoltage(0.0 + leftFeedforward);
        FRMotor.setVoltage(0.0 + rightFeedforward);
        MLMotor.setVoltage(0.0 + leftFeedforward);
        MRMotor.setVoltage(0.0 + rightFeedforward);
        BLMotor.setVoltage(0.0 + leftFeedforward);
        BRMotor.setVoltage(0.0 + rightFeedforward);
    }

    public Boolean reachDriveTarget(Double targetPosition) {
        double averageTickValue = (m_leftEncoder.get() + m_rightEncoder.get()) / 2;

        if (averageTickValue >= 10000) { // if tick value is greater than or equal to 10000, stop both motors
            leftMotors.stopMotor();
            rightMotors.stopMotor();
            return true;
        } else {

        }
        return false;
    }

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MaxSpeed", this::getMaxSpeed, this::setMaxSpeed);
        builder.addFloatArrayProperty("Yaw, Pitch, and Roll Values", this::PrincipalAxisValues, null);
        // Gyro values
        builder.addFloatProperty("X/Roll", this::getXAngle, null);
        builder.addFloatProperty("Y/Pitch", this::getYAngle, null);
        builder.addFloatProperty("Z/Yaw", this::getZAngle, null);
        builder.addDoubleProperty("RightEncoder", this::getRightEncoderTicks, null);
        builder.addDoubleProperty("LeftEncoder", this::getLeftEncoderTicks, null);
        builder.addDoubleProperty("Trajectory Position", this::getTrajPos, null);
        builder.addDoubleProperty("Trajectory Speed", this::getTrajSpeed, null);
        builder.addDoubleProperty("Left Encoder Speed", this::getLeftEncoderSpeed, null);
        builder.addDoubleProperty("Right Encoder Speed", this::getRightEncoderSpeed, null);
        builder.addDoubleProperty("Left Encoder Distance", this::getLeftEncoderDistance, null);
        builder.addDoubleProperty("Right Encoder Distance", this::getRightEncoderDistance, null);

    }
}
