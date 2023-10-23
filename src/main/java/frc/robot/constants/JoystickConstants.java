// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class JoystickConstants {
    private JoystickConstants(){
        throw new IllegalStateException("Constants Class");
    }
    public static final int kRotJoystickPort = 0;
    public static final int kTransJoystickPort = 2;
    public static final int kXboxControllerPort = 1;

    public static final double kDeadZone = 0.2;
    public static final double kRadDeadZone = 0.05;
}