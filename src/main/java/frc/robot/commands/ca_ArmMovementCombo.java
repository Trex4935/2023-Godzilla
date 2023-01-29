// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extensions.ArmPosition;
import frc.robot.extensions.ArmSideOrientation;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.Constants;

public class ca_ArmMovementCombo extends CommandBase {

  private final ArmExtension m_extend;
  private final ArmRotation m_rotate;

  /** Creates a new ca_ArmMovementCombo. */
  public ca_ArmMovementCombo(ArmExtension extend, ArmRotation rotate) {
    m_extend = extend;
    m_rotate = rotate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extend, rotate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**If Robot arm is on CompressorSide (The Front), then run switch case. */

    // Suppresses the ArmMovementCombo if the arm is in the RedZone.
    if (Constants.inRedZone) {
      Constants.selectedArmState = ArmPosition.CARRY;
    }

    if(Constants.selectedArmSideOrientation == ArmSideOrientation.CompressorSide) {
      /** Switch case runs different height presets when selectedArmPosition changes. */
      switch (Constants.selectedArmState) {
        case HIGH:
          m_rotate.AutoArmRotation(Constants.ArmHighAngle);
          m_extend.AutoArmExtension(Constants.ArmHighDistance);
          break;

        case MIDDLE:
          m_rotate.AutoArmRotation(Constants.ArmMiddleAngle);
          m_extend.AutoArmExtension(Constants.ArmMiddleDistance);
          break;

        case LOW:
          m_rotate.AutoArmRotation(Constants.ArmLowAngle);
          m_extend.AutoArmExtension(Constants.ArmLowDistance);
          break;

        default: // Carry Position is default
          m_rotate.AutoArmRotation(Constants.ArmCarryAngle);
          m_extend.AutoArmExtension(Constants.ArmCarryDistance);
      }
    } else {
      switch (Constants.selectedArmState) {
        /** 270 is max rotation, when subtracted gets the mirror angle. 
         * i.e. Low: 0 + 20 |Mirrored| 270 - 20 */
        case HIGH:
          m_rotate.AutoArmRotation(270 - Constants.ArmHighAngle);
          m_extend.AutoArmExtension(Constants.ArmHighDistance);
          break;

        case MIDDLE:
          m_rotate.AutoArmRotation(270 - Constants.ArmMiddleAngle);
          m_extend.AutoArmExtension(Constants.ArmMiddleDistance);
          break;

        case LOW:
          m_rotate.AutoArmRotation(270 - Constants.ArmLowAngle);
          m_extend.AutoArmExtension(Constants.ArmLowDistance);
          break;

        default: // Carry Position is default
          m_rotate.AutoArmRotation(270 - Constants.ArmCarryAngle);
          m_extend.AutoArmExtension(Constants.ArmCarryDistance);
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
