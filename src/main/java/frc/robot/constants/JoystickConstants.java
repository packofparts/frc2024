// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class JoystickConstants {
    private JoystickConstants(){
        throw new IllegalStateException("Constants Class");
    }
    public static final int ROT_JOYSTICK_PORT = 0;
    public static final int TRANS_JOY_PORT = 2;
    public static final int XBOX_PORT = 1;

    public static final double PIVOT_INPUT_DEADZONE = 0.15;
    public static final double EXT_INPUT_DEADZONE = 0.05;

    public static final double PIVOT_INPUT_DESATURATION = 0.05;
    public static final double EXT_INPUT_DESATURATION = 0.17;

    public static final double DRIVE_PRECISION_X_DESATURATION = 3;
    public static final double DRIVE_PRECISION_Y_DESATURATION = 3;
    public static final double DRIVE_PRECISION_ROT_DESATURATION = 6;

    public static final double DRIVE_PRECISION_X_DEADZONE = 0.04;
    public static final double DRIVE_PRECISION_Y_DEADZONE = 0.04;
    public static final double DRIVE_PRECISION_ROT_DEADZONE = 0.02;

    public static final double DRIVE_REG_X_DEADZONE = 0.10;
    public static final double DRIVE_REG_Y_DEADZONE = 0.10;
    public static final double DRIVE_REG_ROT_DEADZONE = 0.15;



}