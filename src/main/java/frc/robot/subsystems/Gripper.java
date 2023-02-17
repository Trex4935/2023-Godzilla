// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Gripper extends SubsystemBase {

    /** Declares new pneumatic objects. */
    Solenoid gripper;
    Compressor compressor;

    public Gripper() {
        /** Creates new pneumatic objects. */
        gripper = new Solenoid(15, PneumaticsModuleType.CTREPCM, 0);
        compressor = new Compressor(15, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
    }

    /** Closes the gripper */
    public void gripClose() {
        // DataLogManager.log("/_\\ GRIPPER CLOSE /_\\");
        gripper.set(true);
    }

    /** Opens the gripper */
    public void gripOpen() {
        // DataLogManager.log("\\_/ GRIPPER OPEN \\_/");
        gripper.set(false);
    }

    /** Turns off the gripper */
    public void disableGrip() {
        DataLogManager.log("|_| GRIPPER OFF |_|");
    }

    public void toggleGrip() {
        gripper.toggle();
    }

    // Sendable override
    // Anything put here will be added to the network tables and thus can be added
    // to the dashboard / consumed by the LED controller
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("GripperState", null, null);
//        builder.addBooleanProperty("isCube", this::getIsCube, null);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
