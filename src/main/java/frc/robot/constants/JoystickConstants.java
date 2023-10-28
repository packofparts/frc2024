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

}