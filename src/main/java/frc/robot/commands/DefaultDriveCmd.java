// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class DefaultDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerveee;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;

  boolean isPrecision = false;

  boolean axisLock = false;
  PIDController axisLockPid;
  double axisLockSetpoint = 0;

  double heading;

  public DefaultDriveCmd(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveee = swerve;
    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccRadPS);
    
    axisLockPid = new PIDController(1, 0, 0);
    axisLockPid.enableContinuousInput(0, Math.PI);
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    heading = (SwerveSubsystem.getHeading() % 180) * Math.PI / 180;

    if(Input.getPrecision()){
      isPrecision = !isPrecision;
    }

    if(Input.doAxisLock()){
      axisLock = !axisLock;
      this.axisLockSetpoint = heading;
      
      if (Math.PI - this.axisLockSetpoint > this.axisLockSetpoint){
        this.axisLockSetpoint = 0;
      }else{
        this.axisLockSetpoint = Math.PI;
      }

    }


    double x = -Input.getJoystickY();
    double y = -Input.getJoystickX();

    SmartDashboard.putNumber("xInput", x);
    SmartDashboard.putNumber("yInput", y);

    SmartDashboard.putBoolean("PrecisionMode", isPrecision);
    SmartDashboard.putBoolean("AxisLock", axisLock);

    double rot = -Input.getRot();

    if(isPrecision){
      rot = rot/2;
      x = x/3;
      y = y/3;
      rot = rot/3;
      x = Math.abs(x) > 0.04 ? x : 0.0;
      y = Math.abs(y) > 0.04 ? y :0.0;
      rot = Math.abs(rot) > 0.02 ? rot : 0.0;
    }else{
      x = Math.abs(x) > 0.10 ? x : 0.0;
      y = Math.abs(y) > 0.10 ? y : 0.0;
      rot = Math.abs(rot) > 0.15 ? rot : 0.0;
    }

    
    if(axisLock){
      rot = axisLockPid.calculate(heading, axisLockSetpoint);
    }



      
    // 3. Make the driving smoother
    x = xLimiter.calculate(x)* DriveConstants.kPhysicalMaxSpeedMPS;
    y = yLimiter.calculate(y)* DriveConstants.kPhysicalMaxSpeedMPS;
    rot = turningLimiter.calculate(rot)* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    swerveee.setMotors(x, y, rot, DriveMode.TELEOP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
  }

  public boolean getClose(){
    boolean gf = true;
    SwerveModule [] rawMods = swerveee.getRawModules();
    for(SwerveModule mod  : rawMods){
      if (!mod.getPIDController().atSetpoint()){
        gf = false;
      }
    }
    return gf;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
