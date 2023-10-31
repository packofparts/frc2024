// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveConfig;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDTuning extends CommandBase {
  private final SwerveSubsystem mSwerve;
  private final SwerveModule mCurModule;
  private final PIDController mCurModPID;
  
  private double mSetPoint = 0;
  private boolean mToggled = false;

  /** Creates a new PIDTuning. */
  public PIDTuning(int modID, SwerveSubsystem swerve) {
    mSwerve = swerve;
    mCurModule = SwerveConfig.SWERVE_MODULES[modID];
    mCurModPID = SwerveConfig.SWERVE_MODULE_PIDs[modID];

    addRequirements(mSwerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI",0);
    SmartDashboard.putNumber("kD",0);
    SmartDashboard.putNumber("increment",2);
    SmartDashboard.putNumber("measurement", mCurModule.getRotPosition()/Math.PI*180);
    SmartDashboard.putNumber("setpoint",mSetPoint);
    SmartDashboard.putBoolean("toggled", mToggled);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double increment = 0;

    if(Input.togglePIDTuning()){
      mToggled = !mToggled;
    }

    if (mToggled){
      mCurModule.setPID(mSetPoint);
    }
    SmartDashboard.putBoolean("toggled", mToggled);
    mCurModPID.setP(SmartDashboard.getNumber("kP", 0));
    mCurModPID.setI(SmartDashboard.getNumber("kI", 0));
    mCurModPID.setD(SmartDashboard.getNumber("kD", 0));
    
    increment = SmartDashboard.getNumber("increment", 2);
    
    if(Input.getIncPID()){
      mSetPoint += increment;
    }else if(Input.getDecPID()){
      mSetPoint -= increment;
    }
    SmartDashboard.putNumber("setpoint", mSetPoint);
    SmartDashboard.putNumber("measurement", mCurModule.getRotPosition()/Math.PI*180);
    SmartDashboard.updateValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // For now we are keeping this empty to handle interruptions
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
