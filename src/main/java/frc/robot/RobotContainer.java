// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.PIDTuning;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.SwerveConstants;
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
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  ArmControlSubsystem m_armControlSubsystem = new ArmControlSubsystem();
  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  Limelight m_limelight = new Limelight();
  PoseEstimation m_pose = new PoseEstimation(m_limelight, m_swerveSubsystem);

  DefaultDriveCommand m_driveCommand = new DefaultDriveCommand(m_swerveSubsystem);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AutoConstants.populateHashMaps(m_armControlSubsystem, m_intakeSubsystem);
    m_armControlSubsystem.setDefaultCommand(new DefaultArmCommand(m_armControlSubsystem));
    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(m_intakeSubsystem));

    if(SwerveConstants.kPIDTuneMode)
      m_swerveSubsystem.setDefaultCommand(new PIDTuning(0, m_swerveSubsystem));
    else
      m_swerveSubsystem.setDefaultCommand(m_driveCommand);
  }



}
