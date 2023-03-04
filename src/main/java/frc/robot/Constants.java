// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;

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

  public static ArmPosition selectedArmState = ArmPosition.CARRY;
  public static ArmSideOrientation selectedArmSideOrientation = ArmSideOrientation.BatterySide;

  // ROBOT Constants
  public static String isCube = "cone";
  // Battery-Side is default
  public static String isBatterySide = "battery";
  public static boolean switchingSides = false;

  // DRIVETRAIN Constants

  // Wheel ID Constants
  public final static int FRMotorID = 1; // RIGHT
  public final static int FLMotorID = 2; // LEFT
  public final static int MRMotorID = 3; // RIGHT
  public final static int MLMotorID = 4; // LEFT
  public final static int BRMotorID = 5; // RIGHT
  public final static int BLMotorID = 6; // LEFT

  /** Measurement Constants */
  public final static int wheelDiameter = 6;
  public final static int encoderTicks = 8192;
  public final static double trackWidth = .641; // Meters

  /** Movement Constants */
  public static final double dtmaxaccel = 1; // Acceleration Limit
  public static double dtmaxspeed = 1; // Speed Limit

  // ARM ROTATION Constants

  /** Motor ID */
  public static final int armRotationCAN = 13;
  public static final double armRotateSpeed = 0.85; // Arm Rotation Speed
  public static double slowArmRotateSpeed = 0.4;
  public static boolean inRedZone = false;

  /** Class for the speed and acceleration limits of the robot. */
  public static class AutoMovementConstraints {
    public static final double dtmaxaccel = 0.75; // 0.5 on flat
    public static final double autodtMaxSpeed = 3.0; // 1 on flat 4.1 m/s is max.
    public static final double dtmaxomega = 1.4;
  }

  /** Motor Rotation Limits */
  public static final int ArmRotationLowerLimit = 0;
  public static final double ArmRotationUpperLimit = 270;

  // Arm Angles
  public static final double ArmHighAngleBattery = 85;
  public static final double ArmMiddleAngleBattery = 70;
  public static final double ArmLowAngleBattery = 16;
  public static double ArmCarryAngleBattery = 10;

  public static final double ArmCarryAngleCompressor = 313;
  public static final double ArmHighAngleCompressor = 240;
  public static final double ArmMiddleAngleCompressor = 250;
  public static final double ArmLowAngleCompressor = 310;

  public static final int sideVal = 0;

  // ARM EXTENSION
  public static final int armExtensionCAN = 21; // Motor CAN ID
  public static final double armExtensionSpeed = 0.7;

  // Arm Distances
  // Units = Inches (Converting to ticks)
  // Math: 1 in. = -5000 ticks
  public static int inchPerExtentionTicks = -5000;
  /** 33 in -> -165000 ticks */
  public static final double ArmHighDistance = -165000;
  /** 10 in -> -50000 */
  public static final double ArmMiddleDistance = -90000;
  /** 20 in -> -100000 */
  public static final double ArmLowDistance = -140000;
  /** 1 in -> -5000 */
  public static final double ArmRammingDistance = 0;

  public static final double ArmCarryDistance = -5000;

  public static final double ArmShelfDistance = -45000;

  public static double tempArmDistance = 10;

  public static boolean buttonOccupied = false;

  // Gripper Constants
  public static boolean gripperClosed = true;

  // DRIVER/CONTROLLER Constants

  // Driver Constants
  // (Differentiates the joysticks inputs by assigning different ID ports)
  public static final int leftJoystick = 0; // Left Joystick ID
  public static final int rightJoystick = 1; // Right Joystick ID

  public static final int joystickTrigger = 1;

  /** Actual Joystick Axis ID */
  public static final int joystickAxis = 1;

  // Controller Constants

  public static final int controllerID = 2; // Controller ID

  // XBOX Controller Constants (SUBJECT TO CHANGE)
  /** Side to side */
  public static final int LeftJoystickX = 0;
  /** Front and back */
  public static final int LeftJoystickY = 1;
  /** Side to side */
  public static final int RightJoystickX = 4;
  /** Front and back */
  public static final int RightJoystickY = 5;

  public static final int LeftTrigger = 2;
  public static final int RightTrigger = 3;

  // Arduino Controller Constants
  // Push Buttons
  public static final int groundButtonID = 1;
  public static final int middleButtonID = 2;
  public static final int highButtonID = 3;
  public static final int shelfButtonID = 13;
  // Joystick
  public static final int ardJoystickUp = 4;
  public static final int ardJoystickDown = 5;
  public static final int ardJoystickLeft = 6;
  public static final int ardJoystickRight = 7;
  // Toggle Switches
  public static final int gamePieceID = 8;
  public static final int robotSideID = 9;
  public static final int gripperID = 10;

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
    public static final double kTrackWidthMeters = 0.641; // 0.584 testing> .65
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static double degreesPerRotationTicks = 1;
  /** Adds extension ticks */
  public static double addExtend = 0;
  public static boolean isAutonomous;

  /** Adds rotation degrees */
  public static double addRotate = 0;

  public static double speedLimit = 0.75;

  // Direction
  public enum direction {
    FRONT,
    BACK,
    RIGHT,
    LEFT
  }

}
