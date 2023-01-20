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

/** Add your docs here. */
public class Gripper extends SubsystemBase{

    /** Declares new pneumatic objects. */
    DoubleSolenoid gripper; // gripper should/could be renamed to be more appropriate i.e., gripOpen, gripClose, etc.
    Compressor compressor;

    public Gripper() {
        /** Creates new pneumatic objects. */
        gripper = new DoubleSolenoid(9, PneumaticsModuleType.CTREPCM, 0, 1);
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    /** Closes the gripper */
    public void gripClose() {
        DataLogManager.log("/_\\ GRIPPER CLOSE /_\\");
        gripper.set(Value.kForward);
    }

    /** Opens the gripper */
    public void gripOpen() {
        DataLogManager.log("\\_/ GRIPPER OPEN \\_/");
        gripper.set(Value.kReverse);
    }

    /** Turns off the gripper */
    public void disableGrip() {
        DataLogManager.log("|_| GRIPPER OFF |_|");
        gripper.set(Value.kOff);
    }
    
}
