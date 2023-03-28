// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  boolean notfound = false;
  public Limelight lime;
  public SwerveSubsystem swerve;
  public PhotonTrackedTarget target;
  public MoveTo move;
  public final double yOffset = 0.36;
  public Transform2d offset;
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
    this.offset = VisionConstants.autoAlign;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhotonPipelineResult img = lime.getImg();

    if (!img.hasTargets()) {
      notfound = true;
    }
    else {
      target = img.getBestTarget();
      // Transform3d transform3d = target.getBestCameraToTarget().plus(VisionConstants.robotToCam);
      Transform3d transform3d = target.getBestCameraToTarget().inverse().plus(VisionConstants.robotToCam).inverse();
      SmartDashboard.putNumber("cameraToTagX", transform3d.getX());
      SmartDashboard.putNumber("cameraToTagY", transform3d.getY());
      Transform2d transform = new Transform2d(new Translation2d(transform3d.getX(), transform3d.getY()), new Rotation2d(transform3d.getRotation().getZ()));
      move = new MoveTo(transform, swerve, pose);
      move.schedule();
    }
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
    if (notfound) {
      return true;
    }
    else {
      return move.isFinished();
    } 
  }


}
