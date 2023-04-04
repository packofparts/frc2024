package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;

/** Add your docs here. */
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



    public static boolean getIncPID(){return tJoystick.getRawButton(5);}
    public static boolean getDecPID(){return rJoystick.getRawButton(4);}
    
    public static boolean getRobotOriented(){return tJoystick.getTrigger();}
    public static boolean resetPose(){return tJoystick.getRawButtonPressed(3);}
    public static boolean limelightAlignTrigger(){return tJoystick.getRawButtonPressed(3);}
    
    //    public static boolean doAimbot() {return false;}
    public static boolean doAimbot() {return tJoystick.getRawButtonPressed(4);}
    public static boolean doAxisLock() {return tJoystick.getRawButtonPressed(2);}
    public static boolean doPrecision(){return rJoystick.getTriggerPressed();}

    public static boolean cancelAllDriveModes() {return tJoystick.getRawButtonPressed(5);}

    //xbox controller 
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
    
    //public static double getRightStickY(){return xboxController.getRightStickY()}
    


}

