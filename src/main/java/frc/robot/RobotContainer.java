// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultArmCommand;
import frc.robot.subsystems.ArmControlSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmControlSubsystem mArmControlSubsystem = new ArmControlSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mArmControlSubsystem.setDefaultCommand(new DefaultArmCommand(mArmControlSubsystem));
  }

  public ArmControlSubsystem getArmSubsystem() {
    return mArmControlSubsystem;
  }
}
