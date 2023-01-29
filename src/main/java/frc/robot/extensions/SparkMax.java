package frc.robot.extensions;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class SparkMax {
    /**
     * Creates a default CAN brushless SparkMax motor. Brushless is the one we use
     * most often.
     * 
     * @param CANID CAN ID of the sparkmax.
     * @return
     */
    public static CANSparkMax createDefaultCANSparkMax(int CANID) {
        return new CANSparkMax(CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    /**
     * Adds SmartMotion functionality and a PID controller
     * @param motorObject
     * The motor object
     * @param kP
     * The proportional gain
     * @param kI
     * The integral gain
     * @param kD
     * The derivative gain
     * @param kFF
     * The feed forward
     * @param minVelocity
     * The minimum velocity desired
     * @param maxVelocity
     * The maximum velocity desired
     * @param maxAcceleration
     * The maximum acceleration desired
     * @param error
     * The amount of closed-loop error allowed
     */
public static CANSparkMax configPIDwithSmartMotion(CANSparkMax motorObject, double kP, double kI, double kD, double kFF, double minVelocity, double maxVelocity, double maxAcceleration, double error) {
    
    /*Creates a PID controller for the SparkMax.*/
    SparkMaxPIDController SparkMaxPID = motorObject.getPIDController();
    
    /*Enables SmartMotion for the PID controller*/
    SparkMaxPID.setReference(0, CANSparkMax.ControlType.kSmartMotion, 0);

    /*Sets the acceleration strat (May not do anything since trapezoidal is the only one now?)*/
    SparkMaxPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    /*Sets motion constraints for the PID controller*/
    SparkMaxPID.setSmartMotionMinOutputVelocity(minVelocity, 0);
    SparkMaxPID.setSmartMotionMaxVelocity(maxVelocity, 0);
    SparkMaxPID.setSmartMotionMaxAccel(maxAcceleration, 0);
    SparkMaxPID.setSmartMotionAllowedClosedLoopError(error, 0);

    /*Gives values to PID controllers*/
    SparkMaxPID.setP(kP);
    SparkMaxPID.setI(kI);
    //Not sure if we need to do something with this
    SparkMaxPID.setIZone(0, 0);
    SparkMaxPID.setD(kD);
    SparkMaxPID.setFF(kFF);

    return motorObject;
}
}