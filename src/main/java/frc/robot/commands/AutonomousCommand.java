// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
  /** Creates a new AutonomousCommand. */
  public Limelight lime;
  public SwerveSubsystem swerve;
  public moveTo move;
  public double desiredOffset = 12;
  public boolean gotOffset;
  public boolean movedForward;
  public boolean turned;
  public AutonomousCommand(Limelight lime, SwerveSubsystem swerve) {
    this.lime = lime;
    this.swerve = swerve;
    gotOffset = false;
    movedForward = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move to Desired Station
    // ---- MoveTo, AutoAlign
    // Turning 180 degrees
    if (!turned) {
      Transform2d transform = new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI));
      if (!move) {
        move = new moveTo(Transform2d transform, SwerveSubsystem swervesub);
        move.schedule();
      }
      else if (move.isFinished()) {
        turned = true;
      }
    }
    else if (!gotOffset) {
      double offset = lime.getXoffset()
      double difference = desiredOffset - offset
      swerve.setMotors(0, 0, difference)
      if (difference<0.2) {
        gotOffset = true;
      }
    }
    else if (!movedForward) {
      transform = new Transform2d(new Translation2d(0, lime.getForwardDistance()), new Rotation2d(0));
      move = new moveTo(transform, swerve);
    }



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
