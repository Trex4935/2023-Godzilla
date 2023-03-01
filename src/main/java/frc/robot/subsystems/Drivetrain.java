// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Gyro Imports
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.extensions.PID;
import frc.robot.Constants.direction;
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

    // PID
    PIDController drivePID;

    // Declaring Gyro Objects
    public static AHRS ahrs;

    // Max speed value for the motors ... default comes from constants
    Double m_MaxSpeed = Constants.dtmaxspeed;

    // Kinemtatics
    DifferentialDriveKinematics kin;
    double trajPos;
    double trajSpeed;

    //
    private final SimpleMotorFeedforward m_feedforward;

    // PID Controller
    private final PIDController m_leftPIDController;
    private final PIDController m_rightPIDController;

    private final PIDController anglePID;

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

        // Create differential drive object
        diffdrive = new DifferentialDrive(leftMotors, rightMotors);
        diffdrive.setMaxOutput(m_MaxSpeed);

        // Create encoders and set distance values
        leftEncoder = new Encoder(1, 2);
        rightEncoder = new Encoder(3, 4);
        leftEncoder.setDistancePerPulse(Units.inchesToMeters(139.565 / 14171));
        rightEncoder.setDistancePerPulse(Units.inchesToMeters(139.565 / 14171)); // 0.00230097
        // Constants.wheelDiameter * Math.PI) / Constants.encoderTicks

        // Create PID Controllers
        m_leftPIDController = new PIDController(0.0225, 0.001, 0);// Right P ==0.01 for flat
        m_rightPIDController = new PIDController(0.0225, 0.001, 0);// Right P ==0.01 for flat
        m_leftPIDController.setIntegratorRange(-12, 12);// V
        m_rightPIDController.setIntegratorRange(-12, 12);// V
        // Feed Forward
        m_feedforward = new SimpleMotorFeedforward(0.18 * 12, 1.15);// new SimpleMotorFeedforward(0.105 * 12, 1.15); //
                                                                    // on the ground 0.165 for ks

        // Creating gyro object
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.calibrate();
        // new Thread(){
        //     public void run(){
        //         Thread.sleep(500);

        //     }.start();
        // }
        // ahrs.reset();

        // Distance between 2 wheel godzilla 641 mm, to do find or measure same for mrT
        kin = new DifferentialDriveKinematics(TrajectoryConstants.kTrackWidthMeters);

        // initiate simulate gyro Position
        zSimAngle = 0;

        drivePID = new PIDController(0.06, 0.0008, 0); // 0.05, 0.001
        drivePID.setIntegratorRange(-4, 4); // -2,2

        anglePID = new PIDController(0.0225, 0.00, 0); // 0.05, 0.001
        drivePID.setIntegratorRange(-180, 180); // -2,2

    }

    /** Resets the gyro */
    public void resetGyro() {
        ahrs.reset();
    }

    /** Gets the offset of the pitch */
    public Float getYAngleOffset() {
        // return ahrs.getYaw();
        return s_getAngleY() - 1.35f;
    }

    /** Gets Yaw(Z) angle from Gyro and converts it to 360 */
    public double getZAngleConverted() {
        double rollBoundedDouble = s_getAngleZ().doubleValue();
        return Helper.ConvertTo360(rollBoundedDouble);
    }

    /** Creates an array of the yaw, pitch, and roll values */
    public float[] PrincipalAxisValues() {
        return new float[] { s_getAngleX(), s_getAngleY(), s_getAngleZ() };
    }

    // Sets motor speed slow for auto.
    public void SlowSpeed() {
        leftMotors.set(0.105);
        rightMotors.set(0.105);
    }

    // Moves the robot with Code
    public void driveWithAuto(double leftSpeed, double rightSpeed) {
        diffdrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void driveWithStraightWithGyro(double avgSpeed, double targetAngle) {
        double err = targetAngle - getZAngleConverted();
        double P = 0.001;
        double driftCorrection = err * P;
        diffdrive.arcadeDrive(avgSpeed, driftCorrection);
    }

    // DEFAULT Command that moves the robot with joysticks
    public void driveWithJoysticks(Joystick joystick1, Joystick joystick2) {
        diffdrive.tankDrive(-joystick1.getRawAxis(Constants.joystickAxis),
                -joystick2.getRawAxis(Constants.joystickAxis));
        // System.out.println("Pitch Angle: " + s_getAngleY());
    }

    /** Stops all Drivetrain motor groups. */
    public void stopMotors() {
        leftMotors.set(0);
        rightMotors.set(0);
    }

    /** Sets the max speed value (sendable) */
    public void setMaxSpeed(double MaxSpeed) {
        Constants.dtmaxspeed = MaxSpeed;
    }

    /** Resets both encoders to 0 */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /** Move the robot based on its pitch/y axis */
    public void autoBalance() {
        // Set motors speed using PID controller to get Y-axis to 0 degrees
        double leftPitch = Math.min(drivePID.calculate(getYAngleOffset(), 0), 0.8);
        double rightPitch = Math.min(drivePID.calculate(getYAngleOffset(), 0), 0.8);
        double err = 0 - s_getAngleZ();
        double P = 0.0175; // 0.2
        double driftCorrectionTwist = err * P;

        leftMotors.set(leftPitch + driftCorrectionTwist); // left: + becase .set, -.setVolt
        rightMotors.set(rightPitch - driftCorrectionTwist); // right: - becase .set, +.setVolt

        System.out.println("leftPitch: " + leftPitch +
                " rightPitch: " + rightPitch + " err: " + err +
                " P: " + P + " driftCorrectionTwist: " + driftCorrectionTwist + " Pitch Angle: " + s_getAngleY());
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
            omega = -AutoMovementConstraints.dtmaxomega;
        } else { // if Start < End, go right, w -
            omega = AutoMovementConstraints.dtmaxomega;
        }

        return omega;
    }

    public void setTrajPos(State currState) {
        trajPos = currState.poseMeters.getY();
    }

    public void setTrajSpeed(State currState) {
        trajSpeed = currState.velocityMetersPerSecond;
    }

    public void driveTankWithStateTraj(State currState, Double end, Double time) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        driveWithAuto(velocityTarget * Math.signum(end), velocityTarget * Math.signum(end));
        // System.out.println("Time:"+ time + "Velocity:" + velocityTarget + "Position:"
        // + currState.poseMeters.getY());
    }

    public void driveTankWithStateKinematicTraj(State currState, Double end, Double time) {
        Double velocityTarget = currState.velocityMetersPerSecond;
        // Rate is 0, because we are following a straight line, the speed varies
        // depending of path, it follows a trapezoid curve.
        Double leftSpeedWheel = getLeftSpeedKin(velocityTarget, 0);
        Double rightSpeedWheel = getRightpeedKin(velocityTarget, 0);
        driveWithAuto(leftSpeedWheel * Math.signum(end), rightSpeedWheel * Math.signum(end));
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

    public Double calculateSpeeds(State currState) {
        // enc pos
        double encoderPosition = (Math.abs(leftEncoder.getDistance()) + Math.abs(rightEncoder.getDistance()))
                / 2;
        // traj pos
        double currentTrajectoryPos = currState.poseMeters.getY();
        double P = 1.0 / 25;
        //
        double targetSpeed = Math.min((currentTrajectoryPos - encoderPosition) / .02 * P,
                AutoMovementConstraints.autodtMaxSpeed);
        return targetSpeed;
    }

    public void driveWithPIDArcade(State currState, Double end, Double time, Double angle) {
        // Double velocityTarget = currState.velocityMetersPerSecond;
        Double velocityTarget = calculateSpeeds(currState) * Math.signum(end);// * Math.signum(end)
        // Rate is 0, because we are following a straight line, the speed varies
        // depending of path, it follows a trapezoide curve.
        Double leftSpeedWheel = -getLeftSpeedKin(velocityTarget, 0);
        Double rightSpeedWheel = -getRightpeedKin(velocityTarget, 0);
        // TO DO
        Double targetAngle = angle;
        double err = targetAngle - s_getAngleZ();
        // if (angle == 0 || angle == 360) {
        // if (getZAngleConverted() <= 180) {
        // err = 0.0 - getZAngleConverted();
        // } else {
        // err = 360 - getZAngleConverted();
        // }
        // }
        double P = 0.03; // 0.02 best value on floor ///0.0175
        //anglePID.calculate(s_getAngleZ(), angle);
        // Deadband
        if (err <= 0.05 && err >= -0.05) {
            err = 0;
        }

        double driftCorrectionTwist = err*P;//anglePID.calculate(s_getAngleZ(), angle); // err * P;
        Double leftSpeedWheelWithGyroCorrection = leftSpeedWheel - driftCorrectionTwist;
        Double rightSpeedWheelWithGyroCorrection = rightSpeedWheel + driftCorrectionTwist;
        //
        setSpeeds(leftSpeedWheelWithGyroCorrection, rightSpeedWheelWithGyroCorrection);

    }

    public void setSpeeds(Double leftSpeedWheel, Double rightSpeedWheel) {
        final double leftFeedforward = m_feedforward.calculate(leftSpeedWheel);
        final double rightFeedforward = m_feedforward.calculate(-rightSpeedWheel);

        final double leftOutput = m_leftPIDController.calculate(leftEncoder.getRate(), leftSpeedWheel); // is encoder
                                                                                                        // in ticks
                                                                                                        // per/sec or
                                                                                                        // m/sec
        final double rightOutput = m_rightPIDController.calculate(rightEncoder.getRate(), -rightSpeedWheel);
        System.out.println("leftSpeed: " + leftSpeedWheel + " rightSpeed: " + rightSpeedWheel + " leftFeedforward: "
                + leftFeedforward + " rightFeedforward: " + rightFeedforward + " leftEncoder :" + leftEncoder.getRate()
                + " rightEncoder: " + rightEncoder.getRate() + " leftOutput: " + leftOutput + " rightOutput: "
                + rightOutput + " leftEncoderDistance: " + leftEncoder.getDistance() + " rightEncoderDistance: "
                + rightEncoder.getDistance());
        FLMotor.setVoltage(leftOutput + leftFeedforward);
        FRMotor.setVoltage(rightOutput + rightFeedforward);
        MLMotor.setVoltage(leftOutput + leftFeedforward);
        MRMotor.setVoltage(rightOutput + rightFeedforward);
        BLMotor.setVoltage(leftOutput + leftFeedforward);
        BRMotor.setVoltage(rightOutput + rightFeedforward);
    }

    public Boolean reachDriveTarget(Double targetPosition) {
        double averageTickValue = (Math.abs(leftEncoder.getDistance()) + Math.abs(rightEncoder.getDistance()))
                * (Math.signum(targetPosition)) / 2;
        System.out.println(" targetPosition: " + targetPosition + " averageDistance: " + averageTickValue);
        if (averageTickValue >= targetPosition - 0.1 && averageTickValue <= targetPosition + 0.1) { // if tick value
                                                                                                    // is greater than
                                                                                                    // or equal to
                                                                                                    // 10000, stop
                                                                                                    // both motors
            leftMotors.stopMotor();
            rightMotors.stopMotor();
            return true;
        } else {

        }
        return false;

    }

    public direction[] calculateTrajEnum(Translation2d goal) {
        direction[] pointMap = {};
        int start = 1;

        // Turn
        if (goal.getX() < 0) {
            pointMap[0] = direction.LEFT;
        } else if (goal.getX() > 0) {
            pointMap[0] = direction.RIGHT;
        } else {
            start = 0;
        }

        // Front
        int j = 0;
        for (int i = start; i < goal.getX(); i++) {
            pointMap[i] = direction.FRONT;
            j = i;
        }
        // Turn
        if (goal.getY() < 0) {
            pointMap[j] = direction.LEFT;
        } else if (goal.getY() > 0) {
            pointMap[j] = direction.RIGHT;
        } else {
            start = j;
        }

        for (int i = start; i < goal.getY(); i++) {
            pointMap[i] = direction.FRONT;
        }

        return pointMap;
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
        return -ahrs.getYaw();
    }

    /** Get the Max speed value (sendable) */
    public double s_getMaxSpeed() {
        return m_MaxSpeed;
    }

    public Double s_getTrajPos() {
        return trajPos;
    }

    public Double s_getTrajSpeed() {
        return trajSpeed;
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

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MaxSpeed", this::s_getMaxSpeed, this::setMaxSpeed);
        // Gyro values
        builder.addFloatProperty("X/Roll", this::s_getAngleX, null);
        builder.addFloatProperty("Y/Pitch", this::s_getAngleY, null);
        builder.addFloatProperty("Z/Yaw", this::s_getAngleZ, null);
        builder.addDoubleProperty("RightEncoder", this::s_getEncoderRightTicks, null);
        builder.addDoubleProperty("LeftEncoder", this::s_getEncoderLeftTicks, null);
        builder.addDoubleProperty("Trajectory Position", this::s_getTrajPos, null);
        builder.addDoubleProperty("Trajectory Speed", this::s_getTrajSpeed, null);
        builder.addDoubleProperty("Left Encoder Speed", this::s_getEncoderLeftSpeed, null);
        builder.addDoubleProperty("Right Encoder Speed", this::s_getEncoderRightSpeed, null);
        builder.addDoubleProperty("Left Encoder Distance", this::s_getEncoderLeftDistance, null);
        builder.addDoubleProperty("Right Encoder Distance", this::s_getEncoderRightDistance, null);

    }
}
