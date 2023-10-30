// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.PIDTuning;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.CompConstants;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  ArmControlSubsystem mArmControlSubsystem = new ArmControlSubsystem();
  IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  Limelight mLimelight = new Limelight();
  PoseEstimation mPoseEstimator = new PoseEstimation(mLimelight, swerveSubsystem);

  DefaultDriveCommand mDriveCommand = new DefaultDriveCommand(swerveSubsystem);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AutoConstants.populateHashMaps(mArmControlSubsystem, mIntakeSubsystem);
    mArmControlSubsystem.setDefaultCommand(new DefaultArmCommand(mArmControlSubsystem));
    mIntakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(mIntakeSubsystem));

    if(CompConstants.PID_TUNE_MODE)
      swerveSubsystem.setDefaultCommand(new PIDTuning(0, swerveSubsystem));
    else
      swerveSubsystem.setDefaultCommand(mDriveCommand);
  }



}
