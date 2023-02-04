// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move to Desired Station
    // ---- MoveTo, AutoAlign

    // Place Piece(Save for later)
    // ---- Needs Arm Commands

    // AprilTag align to find position of piece
    // ---- AprilTag, AutoAlign

    // Returning to Station
    // ---- MoveTo, AutoAlign

    // Placing Cone
    // ---- Arm Commands

    // If Balancing, Balance
    // ---- Balance



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
