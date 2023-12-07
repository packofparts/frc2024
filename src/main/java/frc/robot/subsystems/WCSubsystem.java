// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.subsystems.wcModule;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WCSubsystem extends SubsystemBase {
  private wcModule leftModule;
  private wcModule rightModule;

  private static final double DEFAULT_SPEED = 1; // In inches per second

  /** Creates a new WCSubsystem. */
  /**
   * 
   * @param leftMotorID
   * @param leftEncoderID
   * @param rightMotorID
   * @param rightEncoderID
   */
  public WCSubsystem(int leftMotorID, int leftEncoderID, int rightMotorID, int rightEncoderID) {
    leftModule = new wcModule(leftMotorID, leftEncoderID);
    rightModule = new wcModule(rightMotorID, rightEncoderID);
    leftModule.invert();
  }

  // method that moves the robot forward a certain distance
  public boolean moveRobot(double inches) {
    boolean leftFinished = leftModule.move(inches);
    boolean rightFinished = rightModule.move(inches);

    return (leftFinished || rightFinished);
  }

  public void reset() {
    leftModule.reset();
    rightModule.reset();
  }

  // Not used for now
  public void setRobotVelocity(double velocity) {
    leftModule.maintainVelocity(DEFAULT_SPEED);
    rightModule.maintainVelocity(DEFAULT_SPEED);
  }
}
