// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  public static CANSparkMax armRotationMotor;
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

    // Arm Extension Limit Switches
    armRetractedLimitSwitch = new FlippedDIO(0);

    // Arm Rotation Limit Switches
    compressorSideLimitSwitch = new DigitalInput(6);
    batterySideLimitSwitch = new DigitalInput(5);

    // rotation encoder init
    armRotationEncoder = armRotationMotor.getEncoder();

    // PID for arm extension
    Falcon.configMotionMagic(armExtensionMotor, 0.64, 0, 0, 0, 32000, 32000);

    // PID for arm rotation
    SparkMax.configPIDwithSmartMotion(armRotationMotor, 0.000125, 0.000000, 0.000, 0, 0, 32000, 16000, 0);
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
    if (Helper.RangeCompare(225, 100, armRotationEncoder.getPosition())) {
      Constants.inRedZone = true; // Updates global variable
      Constants.switchingSides = false;
      return true;
    } else {
      Constants.inRedZone = false; // Updates global variable
      return false;
    }
  }

  public void rotateCompressorFast() {
    armRotationMotor.set(0.9);
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

  // Zeros out the entension encoder
  public void zeroEncoder() {
    armExtensionMotor.setSelectedSensorPosition(0, 0, 20);
  }

  // Get the state of the arm extension limit swith
  public boolean getArmRetractedLimitSwitch() {
    return armRetractedLimitSwitch.get();
  }

  // Get the state of the compressor side limit swith
  public boolean getCompressorLimitSwitch() {
    return compressorSideLimitSwitch.get();
  }

  // Get the state of the compressor side limit swith
  public boolean getBatteryLimitSwitch() {
    return batterySideLimitSwitch.get();
  }

  public static boolean checkRotation2(double desiredTicks) {
    desiredTicks = Math.abs(desiredTicks);
    return Helper.RangeCompare(desiredTicks + 1, desiredTicks - 1, Math.abs(armRotationEncoder.getPosition()));
  }

  public static boolean checkExtension2(double desiredTicks) { /** Gets Speed of Arm Extension Motor (Sendable) */
    desiredTicks = Math.abs(desiredTicks);
    return Helper.RangeCompare(desiredTicks + 500, desiredTicks - 500, Math.abs(armExtensionMotor.getSelectedSensorPosition()));
  }

  // ******************** Sendables ********************

  // State of redzone latch
  public boolean s_getLatchEngaged() {
    return redZoneLatch;
  }

  // Amount of extension added to the arm
  private double s_getAddExtend() {
    return Constants.addExtend;
  }

  // Amount of rotation added to the arm
  private double s_getAddRotation() {
    return Constants.addRotate;
  }

  // Report the side that the arm is on
  public String s_getArmSideOrientation() {
    if (Constants.selectedArmSideOrientation == ArmSideOrientation.BatterySide) {
      return "battery";
    } else {
      return "compressor";
    }
  }

  // determine if we are going for a cube or
  public String s_getIsCube() {
    return Constants.isCube;
  }

  // Used to put auto state onto network tables for LEDs
  public String s_getIsAutonomous() {
    if (DriverStation.isAutonomous()) {
      return "GODZILLA";
    } else {
      return "Teleop";
    }

  }

  // Sendable Methods
  /** Gets Encoder Ticks for Extension Encoder (Sendable) */
  public double s_getExtensionEncoderTicks() {
    return armExtensionMotor.getSelectedSensorPosition();
  }

  /** Gets Speed of Arm Extension Motor (Sendable) */
  public double s_getExtensionMotorSpeed() {
    return armExtensionMotor.get();
  }

  /** Returns the encoder value */
  public double s_getRotationEncoderValue() {
    return armRotationEncoder.getPosition();
  }

  /** Sets the default angle value (sendable) */
  public void s_setDefaultAngle(double m_defaultAngle) {
    Constants.ArmCarryAngleBattery = m_defaultAngle;
  }

  /** Get the default angle value (sendable) */
  public double s_getDefaultAngle() {
    return Constants.ArmCarryAngleBattery;
  }

  // Sendable override
  // Anything put here will be added to the network tables and thus can be added
  // to the dashboard / consumed by the LED controller
   @Override
  public void initSendable(SendableBuilder builder) {

    // Arm Extension Sendables
    builder.addDoubleProperty("Extension Encoder Position", this::s_getExtensionEncoderTicks, null);
    builder.addDoubleProperty("Extension Motor Rotation", this::s_getExtensionMotorSpeed, null);
    builder.addBooleanProperty("isRetracted", this::getArmRetractedLimitSwitch, null);
    builder.addDoubleProperty("AddExtension", this::s_getAddExtend, null);

    // Arm Rotation Sendables
    builder.addDoubleProperty("Rotation Encoder", this::s_getRotationEncoderValue, null);
    builder.addDoubleProperty("Default Angle", this::s_getDefaultAngle, this::s_setDefaultAngle);
    builder.addDoubleProperty("AddRotation", this::s_getAddRotation, null);

    // Arm states
    builder.addBooleanProperty("Comp LS", this::getCompressorLimitSwitch, null);
    builder.addBooleanProperty("Batt LS", this::getBatteryLimitSwitch, null);
    builder.addStringProperty("isBatterySide", this::s_getArmSideOrientation, null);
    builder.addStringProperty("isCube", this::s_getIsCube, null);
    builder.addStringProperty("isAutonomous", this::s_getIsAutonomous, null);
    builder.addBooleanProperty("isLatched", this::s_getLatchEngaged, null);
    builder.addBooleanProperty("isRedZone", this::armRedZone, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
