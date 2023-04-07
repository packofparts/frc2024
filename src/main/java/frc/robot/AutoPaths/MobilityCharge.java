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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CompConstants;
import frc.robot.Constants.AutoMapConstants.GamePiece;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.MoveTo;
import frc.robot.commands.armcontrolcmds.ExtensionCmd;
import frc.robot.commands.armcontrolcmds.PivotCmd;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class MobilityCharge extends CommandBase {
  /** Creates a new MobilityAuto. */
  SequentialCommandGroup path;
  public MobilityCharge(SwerveSubsystem swerve, ArmControlSubsystem arm, ClawPnumatic claw) {

    path = new SequentialCommandGroup(
      new InstantCommand(() -> SwerveSubsystem.resetGyro()),
      new PivotCmd(arm, ArmConstants.angleLevelsRad[2]),
      new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[2]),
      new WaitCommand(.3),
      claw.dropPiece(GamePiece.CONE),
      new ExtensionCmd(arm, 0),
      new PivotCmd(arm, ArmConstants.minAngleRad),
      new InstantCommand(() -> SwerveSubsystem.resetGyro(), swerve),
      new InstantCommand(() -> swerve.setMotors(-2.0, 0.0, 0,  DriveMode.AUTO, false), swerve),
      new WaitUntilCommand(() -> SwerveSubsystem.getRoll() <= -CompConstants.onChargeStationOrientation),
      new InstantCommand(() -> swerve.setMotors(-.5, 0.0, 0,  DriveMode.AUTO, false), swerve),
      new WaitCommand(2.5),
      new InstantCommand(() -> swerve.setMotors(0, 0.0, 0,  DriveMode.AUTO, false), swerve),
      new AutoBalanceCommand(swerve)

    );
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, arm, claw);
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
