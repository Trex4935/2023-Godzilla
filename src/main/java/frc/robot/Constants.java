// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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

  // DRIVETRAIN Constants

  /** Wheel ID Constants */
  public final static int FLMotorID = 1; // LEFT
  public final static int FRMotorID = 2; // RIGHT
  public final static int MLMotorID = 3; // LEFT
  public final static int MRMotorID = 4; // RIGHT
  public final static int BLMotorID = 5; // LEFT
  public final static int BRMotorID = 6; // RIGHT

  public static class DrivetrainConstants {
    public final static int wheelDiameter = 6;
    public final static int encoderTicks = 8192;
    public final static double trackWidth = .641; // Meters

  }

  /** Measurement Constants */
  public final static int wheelDiameter = 6;
  public final static int encoderTicks = 8192;

  /** Movement Constants */
  public static final double dtmaxaccel = 1; // Acceleration Limit
  public static final double dtmaxspeed = 0.6; // Speed Limit

  // ARM ROTATION Constants

  /** Motor ID */
  public static final int armRotationCAN = 13;

  /** Class for the speed and acceleration limits of the robot. */
  public static class MovementConstraints {
    public static final double dtmaxaccel = 1;
    public static final double dtmaxspeed = 0.6;
    public static final double dtmaxomega = 0.6;

  }

  /** Motor Rotation Limits */
  public static final int ArmRotationLowerLimit = 0;

  // ARM EXTENSION
  public static final int armExtensionCAN = 33; // Motor CAN ID

  // DRIVER/CONTROLLER Constants

  /** Driver Constants */
  // (Differentiates the joysticks inputs by assigning different ID ports)
  public static final int joystickLeft = 0; // Left Joystick ID
  public static final int joystickRight = 1; // Right Joystick ID

  public static final int joystickAxis = 1; // Actual Joystick Axis ID

  /** Controller Constants */
  public static final int controllerID = 2; // Controller ID

  // XBOX Controller Constants (SUBJECT TO CHANGE)
  public static final int LeftJoystickX = 0; // Side to side
  public static final int LeftJoystickY = 1; // Front and back

  public static final int RightJoystickX = 4; // Side to side
  public static final int RightJoystickY = 5; // Front and back

  public static final int LeftTrigger = 2;
  public static final int RightTrigger = 3;

  /** Class for the Limelight constants. */
  public static class LimeLightConstants {
    public static final double h1 = 1;
    public static final double h2 = 1;
    public static final double angle1 = 1;

  }

  public static final TrapezoidProfile.Constraints thetaConstraints = new Constraints(Constants.dtmaxspeed,
      Constants.dtmaxaccel);

  // Subject to change
  public static class TrajectoryConstants {
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTrackWidthMeters = 0.584; // 0.584 testing> .65
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  // Direction
  public enum direction {
    FRONT,
    BACK,
    RIGHT,
    LEFT
  }

}
