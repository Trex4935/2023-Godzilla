package frc.robot.extensions;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SparkMax {
    /**
     * Creates a default PWMSparkMax Object
     * @param PWMID
     * @return
     */
    public static PWMSparkMax createDefaultPWMSparkMax(int PWMID){
        return new PWMSparkMax(PWMID);
    }

}