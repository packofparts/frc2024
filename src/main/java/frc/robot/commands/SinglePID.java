// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MiscNonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class SinglePID extends CommandBase {
  /** Creates a new SinglePID. */
  SwerveSubsystem swerve;
  SwerveModule selectedModule;
  Double sp;
  public SinglePID(SwerveModule module) {
    this.selectedModule = module;
    SmartDashboard.putNumber("kP", MiscNonConstants.kP);
    SmartDashboard.putNumber("kI", MiscNonConstants.kI);
    SmartDashboard.putNumber("kD", MiscNonConstants.kD);
    SmartDashboard.putNumber("setPointReal", DriveConstants.tuningSetpoint);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("setPointReal", DriveConstants.tuningSetpoint);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    MiscNonConstants.kP = SmartDashboard.getNumber("kP", 0);
    MiscNonConstants.kI = SmartDashboard.getNumber("kI", 0);
    MiscNonConstants.kD = SmartDashboard.getNumber("kD", 0);

    SmartDashboard.putNumber("Setpoint", 0);
    sp = Units.degreesToRadians(SmartDashboard.getNumber("setPointReal", 0));
    SmartDashboard.putNumber("Module1CurrentROT",Units.radiansToDegrees(this.selectedModule.getRotPosition())*DriveConstants.angleEncoderConversionFactortoRad);
    
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
