// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.JoystickConstants;

// Input Class For Joystick/Controller Input Functions
public class Input {

    private static Joystick rJoystick = new Joystick (JoystickConstants.rotJoystickPort);
    private static Joystick tJoystick = new Joystick (JoystickConstants.transJoystickPort);
    
    private static XboxController xboxController = new XboxController(JoystickConstants.xboxControllerPort);

    public static int DPADUP = 0;
    public static int DPADRIGHT = 90;
    public static int DPADDOWN = 180;
    public static int DPADLEFT = 270;


    public static boolean resetGyro(){return rJoystick.getRawButton(3);}

    public static double getJoystickX(){return tJoystick.getX();}

    public static double getJoystickY(){return tJoystick.getY();}

    public static double getRot(){return rJoystick.getX();}

    public static boolean getResetGyro() {return rJoystick.getRawButton(3);}



    public static boolean getIncPID(){return tJoystick.getRawButton(5);}
    public static boolean getDecPID(){return rJoystick.getRawButton(4);}

}
    
