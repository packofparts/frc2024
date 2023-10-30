// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

/** Add your docs here. */
public class IntakeConstants {
    private IntakeConstants(){
        throw new IllegalStateException("Constants Class");
      }
    public static final int INTAKE_ID = 29;
    public static final TalonFXInvertType kIntakeInverted = TalonFXInvertType.Clockwise;
    
    public static final double INPUT_DEADZONE = 0.05;
    public static final double STALL_SPEED = 0.0;

    public static final double TELE_MAX_IN_SPEED_PERCENT = 0.75;
    public static final double TELE_MAX_OUT_SPEED_PERCENT = 0.9;

}
