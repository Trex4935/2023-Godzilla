// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

/** Add your docs here. */
public class Helper {

    /**Compares a value to a maximimum and minimum and return if it is within that range*/

    public static boolean RangeCompare(double maximimum, double minimum, double value) {
        if (value > minimum && value <= maximimum) {
            return true;
        }
        else {
            return false;
        }
        
    }
        


}
