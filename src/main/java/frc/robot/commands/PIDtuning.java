// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MiscNonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDtuning extends CommandBase {
  /** Creates a new PIDtuning. */
  SwerveSubsystem swerveee;
  public PIDtuning(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveee = swerve;
    addRequirements(swerveee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(MiscNonConstants.kP != SmartDashboard.getNumber("p", 0) || MiscNonConstants.kI != SmartDashboard.getNumber("i", 0) ||  MiscNonConstants.kI != SmartDashboard.getNumber("i", 0)){
      MiscNonConstants.kP = SmartDashboard.getNumber("p", 0);
      MiscNonConstants.kI = SmartDashboard.getNumber("i", 0);
      MiscNonConstants.kD = SmartDashboard.getNumber("d", 0);
      if (Input.getIncPID()){
        DriveConstants.tuningSetpoint+=0.1;
      }else if(Input.getDecPID()){
        DriveConstants.tuningSetpoint-=0.1;
      }
      SmartDashboard.putNumber("setPointReal", DriveConstants.tuningSetpoint);
      for (SwerveModule mod: swerveee.getRawModules()){
        mod.updatePositions(DriveConstants.tuningSetpoint);
      }
      
      SmartDashboard.putNumber("Module1CurrentROT",swerveee.getRawModules()[0].getRotPosition());
      SmartDashboard.putNumber("Module2CurrentROT", swerveee.getRawModules()[1].getRotPosition());
      SmartDashboard.putNumber("Module3CurrentROT", swerveee.getRawModules()[2].getRotPosition());
      SmartDashboard.putNumber("Module4CurrentROT", swerveee.getRawModules()[3].getRotPosition());
  }}

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
