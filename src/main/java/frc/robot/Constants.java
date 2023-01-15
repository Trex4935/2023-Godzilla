// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /** Class for the wheel ID constants. */
  public static class WheelIDConstants {
    public final static int FLMotorID = 1;
    public final static int FRMotorID = 2;
    public final static int MLMotorID = 3;
    public final static int MRMotorID = 4;
    public final static int BLMotorID = 5;
    public final static int BRMotorID = 6;
  }

  /** Class for the Limelight constants. */
  public static class LimeLightConstants {
    public static final double h1 = 1;
    public static final double h2 = 1;
    public static final double angle1 = 1;

  }

  /** Class for Xbox Controller ID constants. */
  public static class AxisIDConstants {
    public static final int leftIDAxis = 1;
    public static final int rightIDAxis = 5;

  }

  /** Class for the speed and acceleration limits of the robot. */
  public static class MovementConstraints {
    public static final double dtmaxaccel = 1;
    public static final double dtmaxspeed = 0.75;
    
  }

  // class for Arm Rotation CAN IDs
  public static class ArmRotationConstants {
    public static final int armRotationCAN = 1;
    
  }

  // class for Arm Rotation CAN IDs
  public static class ArmExtensionConstants {
    public static final int armExtensionCAN = 6;
    
  }

  public static class JoystickIDConstants {
    //differentiates the joysticks inputs by assigning different ID ports
    public static final int joystickLeft = 0;
    public static final int joystickRight = 1;
  }

  //Both joysticks use the same Axis value
  public static class JoystickAxis {
    public static final int joystickAxis = 1;
  }
}