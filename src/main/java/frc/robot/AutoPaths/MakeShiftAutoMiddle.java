// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoPaths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoMapConstants.GamePiece;
import frc.robot.commands.AutoBalanceCommand;

import frc.robot.commands.MoveTo;
import frc.robot.commands.armcontrolcmds.ExtensionCmd;
import frc.robot.commands.armcontrolcmds.PivotCmd;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.SwerveSubsystem;

public class MakeShiftAutoMiddle extends CommandBase {
  /** Creates a new MakeShiftAuto. */
  SequentialCommandGroup path;
  public MakeShiftAutoMiddle(ArmControlSubsystem arm, ClawPnumatic claw, SwerveSubsystem swerve) {
    addRequirements(claw);
    addRequirements(arm);
    addRequirements(swerve);

    path = new SequentialCommandGroup(
      
      new InstantCommand(() -> SwerveSubsystem.resetGyro()),
      new PivotCmd(arm, ArmConstants.angleLevelsRad[1]),
      //new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[2]),
      new WaitCommand(.3),
      claw.dropPiece(GamePiece.CONE),
      new PivotCmd(arm, ArmConstants.minAngleRad),
      new WaitCommand(.7),
      new MoveTo(new Transform2d(new Translation2d(-0.3, 0), new Rotation2d(Math.PI)), swerve),
      new InstantCommand(()->SwerveSubsystem.resetGyro()),
      new AutoBalanceCommand(swerve)
    );

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
