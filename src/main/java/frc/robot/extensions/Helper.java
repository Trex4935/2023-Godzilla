// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

/** Add your docs here. */
public class Helper {

    /**
     * Compares a value to a maximum and minimum and return if it is within that
     * range
     */

    public static boolean RangeCompare(double maximum, double minimum, double value) {
        if (value >= minimum && value <= maximum) {
            return true;
        } else {
            return false;
        }

    }

    // https://stackoverflow.com/questions/45316947/converting-between-180-180-to-0-360
    public static double ConvertTo360(double angle) {
        return (angle + 360) % 360;
    }

}
