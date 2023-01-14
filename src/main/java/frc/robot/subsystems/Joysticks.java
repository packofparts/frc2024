package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

/** Add your docs here. */
public class Joysticks {
    Joystick rJoystick;
    Joystick tJoystick;
    public Joysticks(){
        rJoystick = new Joystick (Constants.rotJoystickPort);
        tJoystick = new Joystick (Constants.transJoystickPort);
    }
    public boolean resetGyro(){return rJoystick.getRawButton(3);}
    public double getX(){return tJoystick.getY();}
    public double getY(){return -tJoystick.getX();}
    public double getRot(){return -rJoystick.getX();}
    public boolean getIncPID(){return tJoystick.getRawButton(5);}
    public boolean getDecPID(){return rJoystick.getRawButton(4);}
    public boolean getRobotOriented(){return tJoystick.getTrigger();}
    
}