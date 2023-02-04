// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ca_ArmMovementCombo extends CommandBase {

  private final Arm m_arm;

  /** Creates a new ca_ArmMovementCombo. */
  public ca_ArmMovementCombo(Arm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**If Robot arm is on CompressorSide (The Front), then run switch case. */
    Constants.inRedZone = false;
    // Suppresses the ArmMovementCombo if the arm is in the RedZone.
    if (Constants.inRedZone) {
      Constants.selectedArmState = ArmPosition.CARRY;
    }

    if(Constants.selectedArmSideOrientation == ArmSideOrientation.CompressorSide) {
      /** Switch case runs different height presets when selectedArmPosition changes. */
      switch (Constants.selectedArmState) {
        case HIGH:
          // System.out.println("HIGH-C");
          m_arm.AutoArmRotation(Constants.ArmHighAngleCompressor);
          m_arm.AutoArmExtension(Constants.ArmHighDistance);
          break;

        case MIDDLE:
          // System.out.println("MIDDLE-C");
          m_arm.AutoArmRotation(Constants.ArmMiddleAngleCompressor);
          m_arm.AutoArmExtension(Constants.ArmMiddleDistance);
          break;

        case LOW:
          // System.out.println("LOW-C");
          m_arm.AutoArmRotation(Constants.ArmLowAngleCompressor);
          m_arm.AutoArmExtension(Constants.ArmLowDistance);
          break;

        default: // Carry Position is default
          // System.out.println("DEFAULT-C");
          m_arm.AutoArmRotation(Constants.ArmCarryAngleCompressor);
          m_arm.AutoArmExtension(Constants.ArmCarryDistance);
      }
    } else {
      switch (Constants.selectedArmState) {
        /** 270 is max rotation, when subtracted gets the mirror angle. 
         * i.e. Low: 0 + 20 |Mirrored| 270 - 20 */
        case HIGH:
          // System.out.println("HIGH-B");
          m_arm.AutoArmRotation(Constants.ArmHighAngleBattery);
          m_arm.AutoArmExtension(Constants.ArmHighDistance);
          break;

        case MIDDLE:
          // System.out.println("MIDDLE-B");
          m_arm.AutoArmRotation(Constants.ArmMiddleAngleBattery);
          m_arm.AutoArmExtension(Constants.ArmMiddleDistance);
          break;

        case LOW:
          // System.out.println("LOW-B");
          m_arm.AutoArmRotation(Constants.ArmLowAngleBattery);
          m_arm.AutoArmExtension(Constants.ArmLowDistance);
          break;

        default: // Carry Position is default
          // System.out.println("DEFAULT-B");
          m_arm.AutoArmRotation(Constants.ArmCarryAngleBattery);
          m_arm.AutoArmExtension(Constants.ArmCarryDistance);
      }
    }

    Constants.selectedArmState = ArmPosition.CARRY;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
