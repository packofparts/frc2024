// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;

public class SinglePID extends CommandBase {
  /** Creates a new SinglePID. */
  SwerveSubsystem swerve;
  SwerveModule selectedModule;
  Joysticks joys;
  Double sp;
  public SinglePID(SwerveModule module, SwerveSubsystem swervee) {
    this.selectedModule = module;
    SmartDashboard.putNumber("kP", Constants.kP);
    SmartDashboard.putNumber("kI", Constants.kI);
    SmartDashboard.putNumber("kD", Constants.kD);
    SmartDashboard.putNumber("setPointReal", Constants.tuningSetpoint);
    addRequirements(swervee);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("setPointReal", Constants.tuningSetpoint);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Constants.kP = SmartDashboard.getNumber("kP", 0);
    Constants.kI = SmartDashboard.getNumber("kI", 0);
    Constants.kD = SmartDashboard.getNumber("kD", 0);

    SmartDashboard.putNumber("Setpoint", 0);
    

    sp = SmartDashboard.getNumber("setPointReal", 0)/Constants.rad2Deg;
  
    SmartDashboard.putNumber("Module1CurrentROT",this.selectedModule.getRotPosition()*Constants.rad2Deg*Constants.angleEncoderConversionFactor);
    
    selectedModule.updatePositions(sp);
    
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
