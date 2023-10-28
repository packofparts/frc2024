// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.JoystickConstants;

// Input Class For Joystick/Controller Input Functions
public class Input {
    private Input(){
        throw new IllegalStateException("Input Class");
      }

    private static Joystick rJoystick = new Joystick (JoystickConstants.kRotJoystickPort);
    private static Joystick tJoystick = new Joystick (JoystickConstants.kTransJoystickPort);
    
    private static XboxController xboxController = new XboxController(JoystickConstants.kXboxControllerPort);

    public static final int DPADUP = 0;
    public static final int DPADRIGHT = 90;
    public static final int DPADDOWN = 180;
    public static final int DPADLEFT = 270;


    public static boolean resetGyro(){return rJoystick.getRawButton(3);}
    public static boolean resetOdo() {return tJoystick.getRawButton(3);}

    public static double getJoystickX(){return tJoystick.getX();}

    public static double getJoystickY(){return tJoystick.getY();}

    public static double getRot(){return rJoystick.getX();}

    public static boolean getResetGyro() {return rJoystick.getRawButton(3);}

    public static boolean getPrecisionToggle(){return tJoystick.getTriggerPressed();}

    public static boolean getIncPID(){return rJoystick.getRawButton(5);}
    public static boolean getDecPID(){return rJoystick.getRawButton(4);}
    public static boolean togglePIDTuning(){return rJoystick.getTriggerReleased();}

    public static boolean getA(){return xboxController.getAButtonPressed();}
    public static boolean getB(){return xboxController.getBButtonPressed();}
    public static boolean getX(){return xboxController.getXButtonPressed();}
    public static boolean getY(){return xboxController.getYButtonPressed();}
    public static double getDPad(){return xboxController.getPOV();}
    public static boolean getRightBumper(){return xboxController.getRightBumperPressed();}
    public static boolean getLeftBumper(){return xboxController.getLeftBumper();}

    public static double getLeftTrigger(){return xboxController.getLeftTriggerAxis();}
    public static double getRightTrigger(){return xboxController.getRightTriggerAxis();}

    public static double getLeftStickY(){return -xboxController.getLeftY();}
    public static double getRightStickY(){return -xboxController.getRightY();}

    public static boolean isUltraInstinct() {return xboxController.getStartButtonPressed();}

}
    
