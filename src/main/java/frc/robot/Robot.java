// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    /* To be defined */ 
  }

  @Override
  public void robotPeriodic() {
    /* To be defined */ 
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    /* To be defined */ 
  }

  @Override
  public void disabledPeriodic() {
    /* To be defined */ 
  }

  @Override
  public void autonomousInit() {
    // Enable CANSparkMaxLowLevel.enableExternalUSBControl(true)
    throw new UnsupportedOperationException();
  }

  @Override
  public void autonomousPeriodic() {
    throw new UnsupportedOperationException();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  }

  @Override
  public void teleopPeriodic() { 
    /* To be defined */ 
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
    // Enable _robotContainer.swerveSubsystem.setMotors(0, 0, 0)
  }

  @Override
  public void testPeriodic() {
    // To be defined
  }

  @Override
  public void simulationInit() {
    // To be defined
  }

  @Override
  public void simulationPeriodic() {
    // To be defined
  }
}