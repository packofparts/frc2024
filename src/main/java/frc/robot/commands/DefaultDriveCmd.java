// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerveee;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;
  public DefaultDriveCmd(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveee = swerve;
    xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x= Input.getJoystickX();
    double y = Input.getJoystickX();
    double rot = Input.getRot();
  
    x = Math.abs(x) > 0.15 ? x : 0.0;
    y = Math.abs(y) > 0.15 ? y : 0.0;
    rot = Math.abs(rot) > 0.05 ? rot : 0.0;
      
    // 3. Make the driving smoother
    x = xLimiter.calculate(x) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond;
    y = yLimiter.calculate(y) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond;
    rot= turningLimiter.calculate(rot)
          * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond*3;
    swerveee.setMotors(x, y, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("End");
    //swerveee.goToOrigin();
    //while (!getClose()){
      //swerveee.goToOrigin();
    //}   
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
