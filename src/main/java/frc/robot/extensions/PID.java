// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extensions;

/** Add your docs here. */
public class PID {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    public PID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getkD() {
        return kD;
    }

    public double getkF() {
        return kF;
    }

    public double getkI() {
        return kI;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

}