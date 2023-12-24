// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class Gains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kFF;

    public Gains(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kFF = f;
    }

    public Gains(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        kFF = 0;
    }

    public PIDController toWpiPID() {
        return new PIDController(kP, kI, kD);
    }
}
