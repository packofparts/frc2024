// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class UpdateOffsets extends CommandBase {
  /** Creates a new UpdateOffsets. */
  SwerveSubsystem swerve;
  public UpdateOffsets(SwerveSubsystem swerveSubsystem) {
    this.swerve = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveModule[] modules = swerve.getRawModules();



    for (SwerveModule module : modules) {
      module.setAbsoluteEncoderOffset(module.getAbsolutePosition());
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
