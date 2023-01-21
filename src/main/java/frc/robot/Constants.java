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
    public static final double dtmaxspeed = 0.6;
    
  }

  public static final TrapezoidProfile.Constraints thetaConstraints =
  new Constraints(MovementConstraints.dtmaxspeed, MovementConstraints.dtmaxaccel);

  // class for Arm Rotation CAN IDs
  public static class ArmRotationConstants {
    public static final int armRotationCAN = 33;
    
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
  public static class MotorLimits {
  public static final int ArmRotationConstants = 0;
  }
}

