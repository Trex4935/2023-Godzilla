package frc.robot.extensions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SmartDebug {
    
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table = inst.getTable("debug");

    public static void putBoolean(String name, boolean bol) {
        table.getEntry(name).setBoolean(bol);
    }

    public static void putDouble(String name, double dbl)  {
        table.getEntry(name).setDouble(dbl);
    }

    public static void putString(String name, String str)  {
        table.getEntry(name).setString(str);
    }

}
