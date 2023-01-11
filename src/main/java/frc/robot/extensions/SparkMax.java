package frc.robot.extensions;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

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

}