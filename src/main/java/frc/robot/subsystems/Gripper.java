// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/** Add your docs here. */
public class Gripper extends SubsystemBase{

    /** Declares new pneumatic objects. */
    DoubleSolenoid gripper; // gripper should/could be renamed to be more appropriate i.e., gripOpen, gripClose, etc.
    Compressor compressor;

    public Gripper() {
        /** Creates new pneumatic objects. */
        gripper = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, 0, 1);
        compressor = new Compressor(0, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void gripClose() {
        DataLogManager.log("MESSAGE");
        gripper.set(Value.kForward);
    }

    public void gripOpen() {
        gripper.set(Value.kReverse);
    }
    
}
