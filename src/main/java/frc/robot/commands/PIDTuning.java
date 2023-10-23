// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveConfig;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDTuning extends CommandBase {
  private SwerveSubsystem _driveBase;
  private SwerveModule _curModule;
  private PIDController _curModPID;
  
  private double _setPoint = 0;
  private boolean _toggled = false;

  /** Creates a new PIDTuning. */
  public PIDTuning(int modID, SwerveSubsystem swerve) {
    _driveBase = swerve;
    _curModule = SwerveConfig.swerveModules[modID];
    _curModPID = SwerveConfig.swerveModulePIDs[modID];

    addRequirements(_driveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI",0);
    SmartDashboard.putNumber("kD",0);
    SmartDashboard.putNumber("increment",2);
    SmartDashboard.putNumber("measurement", _curModule.getRotPosition()/Math.PI*180);
    SmartDashboard.putNumber("setpoint",_setPoint);
    SmartDashboard.putBoolean("toggled", _toggled);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double _increment = 0;

    if(Input.togglePIDTuning()){
      _toggled = !_toggled;
    }

    if (_toggled){
      _curModule.setPID(_setPoint);
    }
    SmartDashboard.putBoolean("toggled", _toggled);
    _curModPID.setP(SmartDashboard.getNumber("kP", 0));
    _curModPID.setI(SmartDashboard.getNumber("kI", 0));
    _curModPID.setD(SmartDashboard.getNumber("kD", 0));
    
    _increment = SmartDashboard.getNumber("increment", 2);
    
    if(Input.getIncPID()){
      _setPoint += _increment;
    }else if(Input.getDecPID()){
      _setPoint -= _increment;
    }
    SmartDashboard.putNumber("setpoint", _setPoint);
    SmartDashboard.putNumber("measurement", _curModule.getRotPosition()/Math.PI*180);
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
