package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;

/** Add your docs here. */
public class Input {

    private static boolean useXbox = false; 

    private static Joystick rJoystick = new Joystick (JoystickConstants.rotJoystickPort);
    private static Joystick tJoystick = new Joystick (JoystickConstants.transJoystickPort);


    private static XboxController xboxController = new XboxController(2);

    public static int DPADUP = 0;
    public static int DPADRIGHT = 90;
    public static int DPADDOWN = 180;
    public static int DPADLEFT = 270;


    public static boolean resetGyro(){return rJoystick.getRawButton(3);}
    public static double getJoystickX(){
        return useXbox ? xboxController.getLeftX() : tJoystick.getX();
    }
    public static double getJoystickY(){
        
        return useXbox ? xboxController.getLeftY() : tJoystick.getY();
    
    }
    public static double getRot(){
        return useXbox ? xboxController.getRightX() : rJoystick.getX();
    }
    public static boolean getIncPID(){return tJoystick.getRawButton(5);}
    public static boolean getDecPID(){return rJoystick.getRawButton(4);}
    public static boolean getRobotOriented(){return tJoystick.getTrigger();}
    public static boolean runAutoBalance(){return tJoystick.getRawButton(3);}

    //xbox controller 
    public static boolean getA(){return xboxController.getAButtonPressed();}
    public static boolean getB(){return xboxController.getBButtonPressed();}
    public static boolean getX(){return xboxController.getXButtonPressed();}
    public static boolean getY(){return xboxController.getYButtonPressed();}
    public static double getDPad(){return xboxController.getPOV();}
    public static boolean getRightBumper(){return xboxController.getRightBumper();}
}

