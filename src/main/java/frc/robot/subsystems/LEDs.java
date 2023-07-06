// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.DesiredLED;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDS. */
  Spark led;
  public static double desiredOutput;

  public LEDs() {
    this.led = new Spark(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.set(.83);
  }
  public static void setLEDs(DesiredLED led){
    switch(led){
      case CUBE:
        //yellow
        desiredOutput = 0.69;
      case CONE:
        //violet
        desiredOutput = 0.91;
      default:
        //sky blue
        desiredOutput = 0.83;
    }
  }
}
