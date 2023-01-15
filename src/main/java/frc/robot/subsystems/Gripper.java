// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Gripper extends SubsystemBase{

    /** Creates new pneumatic objects. */
    DoubleSolenoid gripper; // gripper should/could be renamed to be more appropriate i.e., gripOpen, gripClose, etc.
    Compressor compressor;

    public Gripper() {
        gripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    
}
