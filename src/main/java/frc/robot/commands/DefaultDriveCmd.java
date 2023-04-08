// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightPhoton.Pipeline;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class DefaultDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerve;
  LimelightPhoton lime;


  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;

  boolean isPrecision = false;
  boolean isAxisLock = false;
  boolean isConeLock = false;
  boolean isLimeLock = false;
  
  PIDController aimLockPID;
  double axisLockSetpoint = 0;
  double yLockSetpoint = 0;

  double heading;


  PIDController coneLock;
  PIDController limeLockPID;

  public DefaultDriveCmd(SwerveSubsystem swerve) {
    this.swerve = swerve;
    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccRadPS);
    
    aimLockPID = new PIDController(1, 0, 0);
    aimLockPID.enableContinuousInput(0, Math.PI);

    limeLockPID = PIDConstants.YController;

    addRequirements(swerve);
  }

  public DefaultDriveCmd(SwerveSubsystem swerve, LimelightPhoton lime){
    this.swerve = swerve;
    this.lime = lime;
    
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccMPS);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccRadPS);
    
    this.aimLockPID = new PIDController(1, 0, 0);
    this.aimLockPID.enableContinuousInput(0, Math.PI);

    limeLockPID = new PIDController(0.025, 0, 0);
    limeLockPID.setTolerance(1);


    addRequirements(swerve, lime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.heading = (SwerveSubsystem.getHeading() % 180) * Math.PI / 180;

    this.handleInput();


    double x = -Input.getJoystickY();
    double y = -Input.getJoystickX();

    SmartDashboard.putNumber("xInput", x);
    SmartDashboard.putNumber("yInput", y);

    SmartDashboard.putBoolean("PrecisionMode", this.isPrecision);
    SmartDashboard.putBoolean("AxisLock", this.isAxisLock);
    SmartDashboard.putBoolean("ConeLock", this.isConeLock);

    double rot = -Input.getRot();

    if(this.isPrecision){
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

    
    if(this.isAxisLock){
      rot = aimLockPID.calculate(heading, axisLockSetpoint);
    }

    if(this.isConeLock){
      lime.setPipeline(Pipeline.CONE);
      double yaw = this.lime.getYaw() * Math.PI / 180;
      rot = aimLockPID.calculate(yaw, -5 * Math.PI / 180);
    } else{

      if(this.isLimeLock && this.lime.hasTarg()){
        lime.setPipeline(Pipeline.REFLECTION);
        double fy = 0;
        if(Math.abs(y)>0.3 && !limeLockPID.atSetpoint()){
          fy = 0.3* y/Math.abs(y);
        }
        y = limeLockPID.calculate(this.lime.getYaw(),-4)+fy;
      } else{
        
        lime.setPipeline(Pipeline.DRIVE);
      }
    }



      
    // 3. Make the driving smoother
    x = this.xLimiter.calculate(x)* DriveConstants.kPhysicalMaxSpeedMPS;
    y = this.yLimiter.calculate(y)* DriveConstants.kPhysicalMaxSpeedMPS;
    rot = this.turningLimiter.calculate(rot)* DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
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




  void handleInput(){

    if(Input.cancelAllDriveModes()){
      this.isPrecision = false;
      this.isAxisLock = false;
      this.isConeLock = false;
    }


    if(Input.doPrecision()){
      this.isPrecision = !isPrecision;
    }

    if(Input.doAxisLock()){
      this.isAxisLock = !this.isAxisLock;
      this.isConeLock = false;

      this.axisLockSetpoint = heading;
      
      if (Math.PI - this.axisLockSetpoint > this.axisLockSetpoint){
        this.axisLockSetpoint = 0;
      }else{
        this.axisLockSetpoint = Math.PI;
      }
    }

    if(Input.doAimbot()){
      this.isConeLock = !this.isConeLock;
      this.isAxisLock = false;
    }

    if(Input.doLimeLock()){
      this.isLimeLock = true;
    } else{
      this.isLimeLock = false;
    }
  }
}
