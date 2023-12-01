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
  private final SequentialCommandGroup mCommands;

  public Mobility(SwerveSubsystem swerve) {
    addRequirements(swerve);
    mCommands = new SequentialCommandGroup(
        new RunCommand(() -> swerve.setChassisSpeed(-1, 0, 0)).withTimeout(4.2),
        new RunCommand(() -> swerve.setChassisSpeed(0, 0, 0)));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCommands.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mCommands.isFinished();
  }
}
