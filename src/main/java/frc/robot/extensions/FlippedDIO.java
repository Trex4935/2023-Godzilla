// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

import edu.wpi.first.wpilibj.DigitalInput;

// Extend the DigitalInput class
public class FlippedDIO extends DigitalInput {

    public FlippedDIO(int DIOPort) {
        // new up the DI object thru DigitalInput
        super(DIOPort);
    }

    // Return a value but flipped so that active = true
    public boolean get() {
        return !super.get();
    }
}
