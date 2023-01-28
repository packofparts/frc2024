// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCmd extends CommandBase {
  /** Creates a new DefaultDriveCmd. */
  SwerveSubsystem swerveee;
  Joysticks joyee;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turningLimiter;
  public DefaultDriveCmd(Joysticks joys, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joyee = joys;
    this.swerveee = swerve;
    xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond*10);
    yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond*10);
    turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond*30);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x= this.joyee.getX();
    double y = this.joyee.getY();
    double rot = this.joyee.getRot();
  
    x = Math.abs(x) > 0.15 ? x : 0.0;
    y = Math.abs(y) > 0.15 ? y : 0.0;
    rot = Math.abs(rot) > 0.05 ? rot : 0.0;
      
      // 3. Make the driving smoother
    x = xLimiter.calculate(x) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond*1.4;
    y = yLimiter.calculate(y) * Constants.kTeleDriveMaxAccelerationUnitsPerSecond*1.4;
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
