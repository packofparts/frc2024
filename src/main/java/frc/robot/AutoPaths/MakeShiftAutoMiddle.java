// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoPaths;

import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.MoveArm;

import frc.robot.commands.MoveTo;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.SwerveSubsystem;

public class MakeShiftAutoMiddle extends CommandBase {
  /** Creates a new MakeShiftAuto. */

  public MakeShiftAutoMiddle(ArmControlSubsystem arm, ClawPnumatic claw, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    // SequentialCommandGroup path = new SequentialCommandGroup(
    //   new InstantCommand(()->swerve.resetGyro()),
    //   new MoveArmin(arm, ArmConstants.)
    //   new InstantCommand(()->claw.spinOuttake(-0.2)),
    //   new InstantCommand(()->arm.setDesiredPivotRotation(0);)
    //   new MoveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve),
    //   new InstantCommand(()->swerve.resetGyro()),
    //   new AutoBalanceCommand(swerve)
    // );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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
    return false;
  }
}
