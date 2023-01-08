// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Limelight {

  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTable table;

  /** Creates a new Limelight. */
  public Limelight() {
  }

  // Pull TX from the limelight
  public static double getLimelightX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
  }

  // Pull TY from the limelight
  public static double getLimelightY() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

  }

  // Pull tv from the limelight
  public static boolean getLimelightA() {
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0) > 0.075) {
      return true;
    } else {
      return false;
    }
  }

  // Distance to the target
  public static double getDistance() {
    double angle2 = getLimelightY();
    double distance = (Constants.h2 - Constants.h1) / Math.tan(Math.toRadians(Constants.angle1 + angle2)) + 24;
    SmartDashboard.putNumber("Distance Limelight", distance);
    SmartDashboard.putNumber("Angle Limelight", angle2);
    return distance;
  }
}