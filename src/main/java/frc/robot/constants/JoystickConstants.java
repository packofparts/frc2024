// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class JoystickConstants {
    private JoystickConstants(){
        throw new IllegalStateException("Constants Class");
    }
    public static int kRotJoystickPort = 0;
    public static int kTransJoystickPort = 2;
    public static int kXboxControllerPort = 1;

    public static double kDeadZone = 0.2;
    public static double kRadDeadZone = 0.05;
}