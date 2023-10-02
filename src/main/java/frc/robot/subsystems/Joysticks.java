// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joysticks {
  Joystick tStick, rStick;
  public Joysticks() {
    tStick = new Joystick(0);
    rStick = new Joystick(1);
  }

  public boolean resetGyro(){
    return rStick.getRawButton(3);
  }

  public double getX(){
    return -tStick.getX();
  }
  public double getY(){
    return -tStick.getY();
  }

  public double getRot(){
    return -rStick.getX();
  }
  
}
