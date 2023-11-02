// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;


public class DefaultDriveCommand extends CommandBase {
  SwerveSubsystem swerve;
  
  private SlewRateLimiter _xLimiter;
  private SlewRateLimiter _yLimiter;
  private SlewRateLimiter _turningLimiter;
  public DefaultDriveCommand(SwerveSubsystem swerve) {
    addRequirements(swerve);

    _xLimiter = new SlewRateLimiter(SwerveConstants.kTeleMaxSpeedMPS);
    _yLimiter = new SlewRateLimiter(SwerveConstants.kTeleMaxSpeedMPS);
    _turningLimiter = new SlewRateLimiter(SwerveConstants.kTeleMaxRotSpeedRadPerSeconds);
    this.swerve = swerve;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // x, y, and rot are inverted because of the Joystick configurations
    double x = -Input.getJoystickY();
    double y = -Input.getJoystickX();
    double rot = -Input.getRot();
  
    x = Math.abs(x) > 0.10 ? x : 0.0;
    y = Math.abs(y) > 0.10 ? y : 0.0;
    rot = Math.abs(rot) > 0.15 ? rot : 0.0;

    x *= SwerveConstants.kTeleMaxSpeedMPS;
    y *= SwerveConstants.kTeleMaxSpeedMPS;
    rot *= SwerveConstants.kTeleMaxRotSpeedRadPerSeconds;


    SmartDashboard.putNumber("Rotation Janked", rot);
    
    swerve.setMotors(x, y, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // For now we are keeping this empty to handle interruptions in the future
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
