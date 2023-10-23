// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;



public class Mobility extends CommandBase {
  /** Creates a new ScoreMobility. */
  private SwerveSubsystem _swerve;
  private SequentialCommandGroup _commands;
  public Mobility(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this._swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _commands = new SequentialCommandGroup(
      new RunCommand(() -> _swerve.setMotors(-1, 0, 0)).withTimeout(4.2),
      new RunCommand(() -> _swerve.setMotors(0, 0, 0))

    );

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _commands.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _commands.isFinished();
  }
}
