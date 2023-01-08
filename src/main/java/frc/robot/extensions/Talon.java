// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
public class Talon {

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
                20, 60, 0.2);

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
    public static WPI_TalonSRX createDefaultTalon(int id) {
        return createTalon(id, defaultConfig);
    }

    /**
     * Configures a Talon with MotionMagic configuration
     *
     * @param id
     *               CAN ID of the motor to configure
     * 
     * @param config
     *               Config object to use to set values on the motor
     * 
     * @return Configured WPI_TalonFX motor
     */
    public static WPI_TalonSRX createTalon(int id, DefaultConfiguration config) {
        WPI_TalonSRX talon = new WPI_TalonSRX(id);

        talon.configFactoryDefault();
        talon.set(ControlMode.PercentOutput, 0.0);
        talon.setNeutralMode(config.neutralMode);
        talon.configOpenloopRamp(config.openLoopRamp);

        talon.configNominalOutputForward(config.nominalOutputForward);
        talon.configNominalOutputReverse(config.nominalOutputReverse);

        talon.configPeakOutputForward(config.peakOutputForward);
        talon.configPeakOutputReverse(config.peakOutputReverse);

        talon.configSupplyCurrentLimit(config.currLimitCfg);

        // Return the configured motor object
        return talon;
    }

}
