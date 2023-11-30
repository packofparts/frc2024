// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.subsystems.wcModule;

public class WCSubsystem {
  private wcModule leftModule;
  private wcModule rightModule;


  private double PIDOutput;

  private static PIDController pid;

  /** Creates a new WCSubsystem. */
  public WCSubsystem(int leftMotorID, int leftEncoderID, int rightMotorID, int rightEncoderID) {
    leftModule = new wcModule(leftMotorID, leftEncoderID);
    rightModule = new wcModule(rightMotorID, rightEncoderID);
    leftModule._motor.setInverted(true);

  }

  // method that moves the robot forward a certain distance
  public void moveRobot(double inches) {
    leftModule.move(inches);
    rightModule.move(inches);
  }
}
