// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class AimbotDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerve;
  Limelight lemon;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;

  PIDController angleController;
  

  public AimbotDriveCmd(SwerveSubsystem swerve, Limelight lemon) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.lemon = lemon;
    

    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccRadPS);

    angleController = new PIDController(0.75, 0, 0);
    
    addRequirements(swerve, lemon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rot = Input.getRot();
    rot = Math.abs(rot) > 0.05 ? rot : 0.0;
    rot = turningLimiter.calculate(rot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    if(lemon.getBestTarget() != null){
      double yaw = lemon.getXoffset();
      rot = angleController.calculate(yaw, 0);
    }

    double x = Input.getJoystickY();
    double y = Input.getJoystickX();
    
  
    x = Math.abs(x) > 0.1 ? x : 0.0;
    y = Math.abs(y) > 0.1 ? y : 0.0;
    
      
    // 3. Make the driving smoother
    x = xLimiter.calculate(x) * DriveConstants.kPhysicalMaxSpeedMPS;
    y = yLimiter.calculate(y) * DriveConstants.kPhysicalMaxSpeedMPS;
    
    
    this.swerve.setMotors(x, y, rot, DriveMode.TELEOP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
