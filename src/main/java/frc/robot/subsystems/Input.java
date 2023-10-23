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

    private static Joystick _rJoystick = new Joystick (JoystickConstants.kRotJoystickPort);
    private static Joystick _tJoystick = new Joystick (JoystickConstants.kTransJoystickPort);
    
    private static XboxController _xboxController = new XboxController(JoystickConstants.kXboxControllerPort);

    public static final int DPADUP = 0;
    public static final int DPADRIGHT = 90;
    public static final int DPADDOWN = 180;
    public static final int DPADLEFT = 270;


    public static boolean resetGyro(){return _rJoystick.getRawButton(3);}
    public static boolean resetOdo() {return _tJoystick.getRawButton(3);}

    public static double getJoystickX(){return _tJoystick.getX();}

    public static double getJoystickY(){return _tJoystick.getY();}

    public static double getRot(){return _rJoystick.getX();}

    public static boolean getResetGyro() {return _rJoystick.getRawButton(3);}

    public static boolean getPrecisionToggle(){return _tJoystick.getTriggerPressed();}

    public static boolean getIncPID(){return _rJoystick.getRawButton(5);}
    public static boolean getDecPID(){return _rJoystick.getRawButton(4);}
    public static boolean togglePIDTuning(){return _rJoystick.getTriggerReleased();}

    public static boolean getA(){return _xboxController.getAButtonPressed();}
    public static boolean getB(){return _xboxController.getBButtonPressed();}
    public static boolean getX(){return _xboxController.getXButtonPressed();}
    public static boolean getY(){return _xboxController.getYButtonPressed();}
    public static double getDPad(){return _xboxController.getPOV();}
    public static boolean getRightBumper(){return _xboxController.getRightBumperPressed();}
    public static boolean getLeftBumper(){return _xboxController.getLeftBumper();}

    public static double getLeftTrigger(){return _xboxController.getLeftTriggerAxis();}
    public static double getRightTrigger(){return _xboxController.getRightTriggerAxis();}

    public static double getLeftStickY(){return -_xboxController.getLeftY();}
    public static double getRightStickY(){return -_xboxController.getRightY();}

    public static boolean isUltraInstinct() {return _xboxController.getStartButtonPressed();}

}
    
