// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  public Limelight lime;
  public SwerveSubsystem swerve;
  public PhotonTrackedTarget target;
  public moveTo move;
  public final double yOffset = 0.36;
  public Transform2d offset = new Transform2d();
  public Transform2d moveby;

  public PoseEstimation pose;

  Optional<Pose3d> desiredPose3d;

  public AutoAlign(PoseEstimation pose, Limelight limelight, SwerveSubsystem swerve, Transform2d offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pose = pose;
    lime = limelight;
    this.swerve = swerve;
    this.offset = offset;
  }

  public AutoAlign(PoseEstimation pose, Limelight limelight, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pose = pose;
    lime = limelight;
    this.swerve = swerve;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (move == null) {
      target = lime.getBestTarget();
      if (target != null) {
        swerve.stopAllAndBrake();
        desiredPose3d = pose.layout.getTagPose(target.getFiducialId());

        Pose2d desiredPose = new Pose2d(desiredPose3d.get().getX(), desiredPose3d.get().getY()+yOffset, new Rotation2d(desiredPose3d.get().getRotation().getAngle()));
        moveby = pose.getPosition().minus(desiredPose.plus(offset));
        move = new moveTo(moveby, swerve);
        move.schedule();
      } else {
        swerve.setMotors(0, 0, 0.1);
      }
    } 
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
