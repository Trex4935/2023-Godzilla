// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/** Add your docs here. */
public class Falcon {

    private static final int kTimeout = 20;

    public static class DefaultConfiguration {

        // Set Motor Brake mode
        public NeutralMode neutralMode = NeutralMode.Brake;

        // Input dead band
        public double neutralDeadband = 0.04;

        // Motor ramp when using open loop
        public double openLoopRamp = 0.75;

        // Set motor limits
        //// normal output forward and reverse = 0% ... i.e. stopped
        public double nominalOutputForward = 0;
        public double nominalOutputReverse = 0;

        //// Max output forward and reverse = 100%
        public double peakOutputForward = 1;
        public double peakOutputReverse = -1;

        public boolean enableCurrentLimit = false;
        public SupplyCurrentLimitConfiguration currLimitCfg = new SupplyCurrentLimitConfiguration(enableCurrentLimit,
                20, 60, .2);

        // When setting up the sensor boot to a value of zero
        public SensorInitializationStrategy sensorInitializationStrategy = SensorInitializationStrategy.BootToZero;

        // Encoder based limit switches
        public boolean enableSoftLimit = false;
        public int forwardSoftLimit = 0;
        public int reverseSoftLimit = 0;

    }

    private static DefaultConfiguration defaultConfig = new DefaultConfiguration();

    //
    /**
     * create a CANTalon with the default (out of the box) configuration
     *
     * @param id
     *           CAN ID of the motor to configure
     * 
     * @return Configured WPI_TalonFX motor
     */
    public static WPI_TalonFX createDefaultFalcon(int id) {
        return createFalcon(id, defaultConfig);
    }

    /**
     * Configures a Falcon with MotionMagic configuration
     *
     * @param id
     *               CAN ID of the motor to configure
     * 
     * @param config
     *               Config object to use to set values on the motor
     * 
     * @return Configured WPI_TalonFX motor
     */
    public static WPI_TalonFX createFalcon(int id, DefaultConfiguration config) {
        WPI_TalonFX falcon = new WPI_TalonFX(id);

        falcon.configFactoryDefault();
        falcon.set(ControlMode.PercentOutput, 0.0);
        falcon.setNeutralMode(config.neutralMode);
        falcon.configOpenloopRamp(config.openLoopRamp);

        falcon.configNominalOutputForward(config.nominalOutputForward);
        falcon.configNominalOutputReverse(config.nominalOutputReverse);

        falcon.configPeakOutputForward(config.peakOutputForward);
        falcon.configPeakOutputReverse(config.peakOutputReverse);

        falcon.configSupplyCurrentLimit(config.currLimitCfg);

        falcon.configIntegratedSensorInitializationStrategy(config.sensorInitializationStrategy);

        // Configured encoder based limit switch
        falcon.configForwardSoftLimitEnable(config.enableSoftLimit);
        falcon.configReverseSoftLimitEnable(config.enableSoftLimit);
        falcon.configForwardSoftLimitThreshold(config.forwardSoftLimit);
        falcon.configReverseSoftLimitThreshold(config.reverseSoftLimit);

        // Return the configured motor object
        return falcon;
    }

    /**
     * Configures a Falcon with a PID configuration in the 0 slot
     *
     * @param motorObject
     *                    WPI_TalonFX object to configure
     * @param kP
     *                    Double value of kP portion of PID
     * 
     * @param kI
     *                    Double value of kI portion of PID
     * 
     * @param kD
     *                    Double value of kD portion of PID
     * 
     * @param kF
     *                    Double value of KF portion of PID
     * 
     * 
     * @return Configured WPI_TalonFX motor with PID in ID 0
     */
    public static WPI_TalonFX configurePID(WPI_TalonFX motorObject, double kP, double kI, double kD, double kF) {

        // PID configs
        // setting up the pid
        motorObject.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeout);
        // Set kP(proportional); kI(Integral); kD(differential); kF(FeedForward)
        motorObject.config_kP(0, kP, kTimeout);
        motorObject.config_kI(0, kI, kTimeout);
        motorObject.config_kD(0, kD, kTimeout);
        motorObject.config_kF(0, kF, kTimeout);

        motorObject.config_IntegralZone(0, 0, kTimeout);

        return motorObject;
    }

    /**
     * Configures a Falcon with MotionMagic configuration
     *
     * @param motorObject
     *                       WPI_TalonFX object to configure
     * @param kP
     *                       Double value of kP portion of PID
     * 
     * @param kI
     *                       Double value of kI portion of PID
     * 
     * @param kD
     *                       Double value of kD portion of PID
     * 
     * @param kF
     *                       Double value of KF portion of PID
     * 
     * @param CruiseVelocity
     *                       Double cruise speed of motor during motion
     * 
     * @param Acceleration
     *                       Double acceleration to get to cruise speed
     * 
     * @return Configured WPI_TalonFX motor
     */
    public static WPI_TalonFX configMotinoMagic(WPI_TalonFX motorObject, double kP, double kI, double kD, double kF,
            double CruiseVelocity, double Acceleration) {

        // Auxilary motor
        motorObject.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                kTimeout);
        motorObject.configNeutralDeadband(0.001, kTimeout);
        motorObject.configOpenloopRamp(1);

        /* Set Motion Magic gains in slot0 - see documentation */
        motorObject.selectProfileSlot(0, 0);
        motorObject.config_kF(0, kF, kTimeout);
        motorObject.config_kP(0, kP, kTimeout);
        motorObject.config_kI(0, kI, kTimeout);
        motorObject.config_kD(0, kD, kTimeout);

        /* Set acceleration and vcruise velocity - see documentation */
        motorObject.configMotionCruiseVelocity(CruiseVelocity, kTimeout);
        motorObject.configMotionAcceleration(Acceleration, kTimeout);

        /* Zero the sensor once on robot boot up */
        motorObject.setSelectedSensorPosition(0, 0, kTimeout);

        /* integral Zone */
        motorObject.config_IntegralZone(0, 200);

        return motorObject;
    }
}
