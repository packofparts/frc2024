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
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem();
  private final ArmControlSubsystem mArmControlSubsystem = new ArmControlSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final Limelight mLimelight = new Limelight();
  private final PoseEstimation mPoseEstimator = new PoseEstimation(mLimelight, mSwerveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AutoConstants.populateHashMaps(mArmControlSubsystem, mIntakeSubsystem);
    mArmControlSubsystem.setDefaultCommand(new DefaultArmCommand(mArmControlSubsystem));
    mIntakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(mIntakeSubsystem));

    if (CompConstants.PID_TUNE_MODE)
      mSwerveSubsystem.setDefaultCommand(new PIDTuning(0, mSwerveSubsystem));
    else
      mSwerveSubsystem.setDefaultCommand(new DefaultDriveCommand(mSwerveSubsystem));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return mSwerveSubsystem;
  }

  public ArmControlSubsystem getArmSubsystem() {
    return mArmControlSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return mIntakeSubsystem;
  }

  public Limelight getLimelight() {
    return mLimelight;
  }

  public PoseEstimation getPoseEstimator() {
    return mPoseEstimator;
  }
}
