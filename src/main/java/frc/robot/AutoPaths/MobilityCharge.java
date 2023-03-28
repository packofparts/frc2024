// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoPaths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.MoveTo;
import frc.robot.subsystems.SwerveSubsystem;

public class MobilityCharge extends CommandBase {
  /** Creates a new MobilityAuto. */
  SequentialCommandGroup path;
  public MobilityCharge(SwerveSubsystem swerve) {

    path = new SequentialCommandGroup(
      new InstantCommand(swerve::resetGyro),
      new MoveTo(new Transform2d(new Translation2d(-3, 0), new Rotation2d()), swerve),
      new WaitCommand(1),
      new AutoBalanceCommand(swerve)

    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return path.isFinished();
  }
}
