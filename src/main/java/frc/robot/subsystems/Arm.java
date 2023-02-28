// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.extensions.Falcon;
import frc.robot.extensions.FlippedDIO;
import frc.robot.extensions.Helper;
import frc.robot.extensions.SparkMax;

public class Arm extends SubsystemBase {

  // Arm Extension
  private static WPI_TalonFX armExtensionMotor;

  public DigitalInput armRetractedLimitSwitch;

  // Arm Rotation
  public CANSparkMax armRotationMotor;
  static RelativeEncoder armRotationEncoder;

  SparkMaxPIDController armRotationPID;

  DigitalInput compressorSideLimitSwitch;
  DigitalInput batterySideLimitSwitch;

  public static boolean redZoneLatch = false;

  /** Creates a new Arm. */
  public Arm() {
    // init motors
    armExtensionMotor = Falcon.createDefaultFalcon(Constants.armExtensionCAN);
    armRotationMotor = SparkMax.createDefaultCANSparkMax(Constants.armRotationCAN);
    armRotationPID = armRotationMotor.getPIDController();

    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
    armRotationMotor.setIdleMode(IdleMode.kBrake);

    // Arm Extension Limit Switches
    armRetractedLimitSwitch = new FlippedDIO(0);
    // Arm Rotation Limit Switches
    compressorSideLimitSwitch = new DigitalInput(6);
    batterySideLimitSwitch = new DigitalInput(5);

    // rotation encoder init
    armRotationEncoder = armRotationMotor.getEncoder();
    Falcon.configMotionMagic(armExtensionMotor, 0.01, 0, 0, 0, 32000, 32000);
    SparkMax.configPIDwithSmartMotion(armRotationMotor, 0.0001, 0, 0, 0, 0, 32000, 16000, 0);

  }

  // Arm Extension Methods
  /** Stops the Extension Motor */
  public void stopExtensionMotor() {
    armExtensionMotor.stopMotor();
  }

  /** Stops the Rotation Motor */
  public void stopRotationMotor() {
    armRotationMotor.stopMotor();
  }

  // Rotates the arm towards the chassis until limit switch is tripped
  public void armRotationToLimit(ArmSideOrientation m_armSide) {
    // If batterySide, move to limit switch
    if (m_armSide == ArmSideOrientation.BatterySide) {
      if (batterySideLimitSwitch.get()) {
        armRotationMotor.stopMotor();
      } else {
        armRotationMotor.set(-0.4);
      }
    }
    // If compressorSide, move to limit switch
    else {
      if (compressorSideLimitSwitch.get()) {
        stopRotationMotor();
      } else {
        armRotationMotor.set(0.4);
      }
    }
  }

  /** Using MotionMagic set the arm to a given position */
  public void setArmExtensionMM(double armPositionTicks) {
    armExtensionMotor.set(TalonFXControlMode.MotionMagic, armPositionTicks + Constants.addExtend);
  }

  /** Using SmartMotion to set the arm to a given angle */
  public void setArmRotationSM(double armRotationTicks) {
    // IF in REDZONE or not retracted, ENGAGE LATCH.
    if (armRedZone() && !getArmRetractedLimitSwitch()) {
      redZoneLatch = true;
    }

    // if latched, STOP MOTOR.
    if (redZoneLatch) {
      armRotationMotor.stopMotor();

      // if latched but fully retracted, DISENGAGE LATCH.
      if (getArmRetractedLimitSwitch()) {
        redZoneLatch = false;
      }

    }
    // if not latched or hit limit switch, MOVE MOTOR.
    else {
      armRotationPID.setReference(armRotationTicks + Constants.addRotate, ControlType.kSmartMotion);
    }
  }

  /** Sets the speed that the arm moves backward */
  public void retractArm() {
    if (armRetractedLimitSwitch.get()) {
      // if the backwardimitSwitch is true,stop the motor
      armExtensionMotor.stopMotor();
      zeroEncoder();
    } else {
      // if the backwardLimitSwitch is false, then allow the motor to keep moving
      armExtensionMotor.set(Constants.armExtensionSpeed);
    }
  }

  // Arm Rotation Methods
  /**
   * determines if the arm is in the red zone or not, and if it is extended or not
   */
  public boolean armRedZone() {
    // if arm is in red zone and it is extended
    if (Helper.RangeCompare(225, 91, armRotationEncoder.getPosition())) {
      Constants.inRedZone = true; // Updates global variable
      Constants.switchingSides = false;
      return true;
    } else {
      Constants.inRedZone = false; // Updates global variable
      return false;
    }
  }

  /** Increases addExtend */
  public void increaseTicks() {
    Constants.addExtend -= 500;
  }

  /** Decreases addExtend */
  public void decreaseTicks() {
    Constants.addExtend += 500;
  }

  /** resets the addExtend value */
  public void resetExtensionAdditionTicks() {
    Constants.addExtend = 0;
  }

  /** Rotates the arm towards the compressor by 0.5 degrees */
  public void manualRotateCompressor() {
    Constants.addRotate += 0.5;
  }

  /** Rotates the arm towards the battery by 0.5 degrees */
  public void manualRotateBattery() {
    Constants.addRotate -= 0.5;
  }

  /** Resets additional rotation angles */
  public void resetAddRotationAngle() {
    Constants.addRotate = 0;
  }

  // Sendable Methods
  /** Gets Encoder Ticks for Extension Encoder (Sendable) */
  public double getExtensionEncoderTicks() {
    return armExtensionMotor.getSelectedSensorPosition();
  }

  /** Gets Speed of Arm Extension Motor (Sendable) */
  public double getExtensionMotorSpeed() {
    return armExtensionMotor.get();
  }

  public String getExtensionPosition() {

    return "Error";
  }

  public void zeroEncoder() {
    armExtensionMotor.setSelectedSensorPosition(0, 0, 20);
  }

  /** Returns the encoder value */
  public double getEncoderValue() {
    return armRotationEncoder.getPosition();
  }

  /** Returns the angle */
  public double getArmAngle() {
    return getEncoderValue();
  }

  /** Sets the default angle value (sendable) */
  public void setDefaultAngle(double m_defaultAngle) {
    Constants.ArmCarryAngleBattery = m_defaultAngle;
  }

  /** Get the default angle value (sendable) */
  public double getDefaultAngle() {
    return Constants.ArmCarryAngleCompressor;
  }

  /** Method that determines if the arm is retracted or not */
  public boolean getArmRetractedLimitSwitch() {
    return armRetractedLimitSwitch.get();
  }

  public boolean getCompressorLimitSwitch() {
    return compressorSideLimitSwitch.get();
  }

  public boolean getBatteryLimitSwitch() {
    return batterySideLimitSwitch.get();
  }

  public void setArmLength(Double m_tempArmDistance) {
    Constants.tempArmDistance = m_tempArmDistance;
  }

  public double getArmLength() {
    return Constants.tempArmDistance;
  }

  public String getArmSideOrientation() {
    if (Constants.selectedArmSideOrientation == ArmSideOrientation.BatterySide) {
      return "battery";
    } else {
      return "compressor";
    }
  }

  public String getIsCube() {
    return Constants.isCube;
  }

  public static boolean checkRotation(ArmPosition armPos) {

    boolean isReached = false;
    // battery side
    double ticksOffHighB = armRotationEncoder.getPosition() - Constants.ArmHighAngleBattery;
    double ticksOffMiddleB = armRotationEncoder.getPosition() - Constants.ArmMiddleAngleBattery;
    double ticksOffLowB = armRotationEncoder.getPosition() - Constants.ArmLowAngleBattery;
    double ticksOffCarryB = armRotationEncoder.getPosition() - Constants.ArmCarryAngleBattery;
    // compressor side
    double ticksOffHighC = armRotationEncoder.getPosition() - Constants.ArmHighAngleCompressor;
    double ticksOffMiddleC = armRotationEncoder.getPosition() - Constants.ArmMiddleAngleCompressor;
    double ticksOffLowC = armRotationEncoder.getPosition() - Constants.ArmLowAngleCompressor;
    double ticksOffCarryC = armRotationEncoder.getPosition() - Constants.ArmCarryAngleCompressor;
    // if battery side, check position and if its reached the target
    if (Constants.selectedArmSideOrientation == ArmSideOrientation.BatterySide) {

      switch (armPos) {
        case HIGH:
          isReached = ticksOffHighB <= 1 && ticksOffHighB >= -1;
          System.out.println("highb reached");
          break;

        case MIDDLE:
          isReached = ticksOffMiddleB <= 1 && ticksOffMiddleB >= -1;
          System.out.println("middleb reached");
          break;

        case LOW:
          isReached = ticksOffLowB <= 1 && ticksOffLowB >= -1;
          System.out.println("lowb reached");
          break;

        case CARRY:
          isReached = ticksOffCarryB <= 1 && ticksOffCarryB >= -1;
          System.out.println("carryb reached");
          break;

        default:
          isReached = false;
          break;
      }

      return isReached;

      // if compressor side, check position and if its reached the target
    } else {

      switch (armPos) {
        case HIGH:
          isReached = ticksOffHighC <= 1 && ticksOffHighC >= -1;
          System.out.println("highc reached");
          break;

        case MIDDLE:
          isReached = ticksOffMiddleC <= 1 && ticksOffMiddleC >= -1;
          System.out.println("middlec reached");
          break;

        case LOW:
          isReached = ticksOffLowC <= 1 && ticksOffLowC >= -1;
          System.out.println("lowc reached");
          break;

        case CARRY:
          isReached = ticksOffCarryC <= 1 && ticksOffCarryC >= -1;
          System.out.println("carryc reached");
          break;

        default:
          isReached = false;
          break;
      }
      return isReached;
    }
  }

  public static boolean checkExtension(ArmPosition armPos) {

    boolean isExtended = false;
    double ticksOffHigh = armExtensionMotor.getSelectedSensorPosition() - Constants.ArmHighDistance;
    double ticksOffMiddle = armExtensionMotor.getSelectedSensorPosition() - Constants.ArmMiddleDistance;
    double ticksOffLow = armExtensionMotor.getSelectedSensorPosition() - Constants.ArmLowDistance;
    double ticksOffCarry = armExtensionMotor.getSelectedSensorPosition() - Constants.ArmCarryDistance;

    switch (armPos) {
      case HIGH:
        isExtended = ticksOffHigh <= 500 && ticksOffHigh >= -500;
        break;

      case MIDDLE:
        isExtended = ticksOffMiddle <= 500 && ticksOffMiddle >= -500;
        break;

      case LOW:
        isExtended = ticksOffLow <= 500 && ticksOffLow >= -500;

      case CARRY:
        isExtended = ticksOffCarry <= 500 && ticksOffCarry >= -500;

      default:
        isExtended = false;
        break;
    }

    return isExtended;

  }

  public String getIsAutonomous() {
    if (DriverStation.isAutonomous()) {
      return "GODZILLA";
    } else {
      return "Teleop";
    }

  }

  public boolean getLatchEngaged() {
    return redZoneLatch;
  }
  /*
   * ====MATH====
   * Ticks per rotation, 42
   * Gear Ratio Reduction, 144:1
   * Gear has 40 teeth, sprocket has 12
   * 12 degrees per tooth
   * 144 degrees-?
   * 0.83 degrees per rotation = 42 ticks?
   * .002 degrees per tick-?
   * 
   * fixed math 2/4/23
   * small gear has 15 (40/15 ratio = 2.67), big circle (gear?) has 40...
   * apparently its 9 degrees per tooth now?
   * every 6048 encoder ticks, small gear turns 2.67 times, big gear moves once
   * every 44.86 ticks, we move one degree
   * 144 x 42 = 6048
   * 6048 x 2.67 = 16148.16
   * 16148.16 / 360 = 44.856 rounded 44.86
   * 
   */

  /*
   * Preset Arm Positions during Auto
   * (Takes in angle we need to get to) - (current encoder value) = angle we need
   * to move
   * -> angle we need to move * 500 = ticks we need to move
   * EXAMPLE: (90 degrees) - (0 degrees) = 90 degrees ... 90 degrees * 500 = 4500
   * ticks
   * ->>> Then the rotation motor is moved until it reaches 4500 ticks.
   */

  private double getAddExtend() {
    return Constants.addExtend;
  }

  // Sendable override
  // Anything put here will be added to the network tables and thus can be added
  // to the dashboard / consumed by the LED controller
  @Override
  public void initSendable(SendableBuilder builder) {

    // Arm Extension Sendables
    builder.addDoubleProperty("Extension", null, null);
    builder.addDoubleProperty("Extension Encoder Position", this::getExtensionEncoderTicks, null);
    builder.addDoubleProperty("Extension Motor Rotation", this::getExtensionMotorSpeed, null);
    builder.addStringProperty("Arm Extension Position", this::getExtensionPosition, null);
    builder.addBooleanProperty("Is Retracted", this::getArmRetractedLimitSwitch, null);
    builder.addDoubleProperty("Arm Length", this::getArmLength, this::setArmLength);
    builder.addDoubleProperty("AddExtension", this::getAddExtend, null);

    // Arm Rotation Sendables
    builder.addDoubleProperty("Angle", this::getArmAngle, null);
    builder.addDoubleProperty("Rotation Encoder", this::getEncoderValue, null);
    builder.addBooleanProperty("RedZone", this::armRedZone, null);
    builder.addDoubleProperty("Default Angle", this::getDefaultAngle, this::setDefaultAngle);

    builder.addBooleanProperty("Comp LS", this::getCompressorLimitSwitch, null);
    builder.addBooleanProperty("Batt LS", this::getBatteryLimitSwitch, null);
    builder.addStringProperty("isBatterySide", this::getArmSideOrientation, null);
    builder.addStringProperty("isCube", this::getIsCube, null);
    builder.addStringProperty("isAutonomous", this::getIsAutonomous, null);

    builder.addBooleanProperty("RedZone Latch Engaged", this::getLatchEngaged, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
// Arm Rotation Notes
// 0, 1, 5, 50 , 60 (Normal arm side degrees)
// 270, 269, 265, 220, 210 (Opposite arm side degrees)
