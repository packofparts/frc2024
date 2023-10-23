// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kIntakeID = 6;
    public static final TalonFXInvertType kIntakeInverted = TalonFXInvertType.Clockwise;
    
    public static final double kIntakeDeadZone = 0.05; //.05
    public static final double kIntakeStallSpeed = 0.0; //.1
}
