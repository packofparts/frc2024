// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  public Limelight lime;
  public SwerveSubsystem swerve;
  public PhotonTrackedTarget target;
  public moveTo move;
  public final double yOffset = 0.36;


  public AutoAlign(SwerveSubsystem swerve, Limelight lime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = lime;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = lime.getBestTarget();
    Transform3d transformation = target.getBestCameraToTarget();
    Transform2d transform2d = new Transform2d(new Translation2d(transformation.getX(), transformation.getY()), new Rotation2d(transformation.getRotation().getAngle()));


    move = new moveTo(transform2d.inverse(), swerve);
    move.schedule();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return move.isFinished();
  }


}
