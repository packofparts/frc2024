// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import javax.swing.text.Utilities;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.Constants.VisionConstants;


public class ManualPoseEstimation extends SubsystemBase {
  /** Creates a new PoseEstimation. */
  public AprilTagFieldLayout layout;
  public PhotonPoseEstimator estimator;
 // Transformation from robot to 
 SwerveSubsystem swerve;
 Limelight lime;
 SwerveDrivePoseEstimator poseEstimator;
 double visionTimestamp;
 
  public ManualPoseEstimation(Limelight limelight, SwerveSubsystem swerve) {
    this.swerve = swerve;
    lime = limelight;
    try {
      layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("biggestbird.json"));
    } catch (IOException e) {
      e.printStackTrace();
    }



    poseEstimator = new SwerveDrivePoseEstimator(swerve.m_kinematics, swerve.getRotation2d(), swerve.getModulePositions(), getOdometry());

  }

  public Pose2d getVision() {
    PhotonTrackedTarget target = lime.getBestTarget();
    visionTimestamp = lime.getTimestamp();
    if (target != null) {
      if (target.getPoseAmbiguity()<=.2) {
        Pose3d pose = layout.getTagPose(target.getFiducialId()).get().plus(target.getBestCameraToTarget().plus(VisionConstants.robotToCam));
        Pose2d pose3disdumb = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getAngle()));
        return pose3disdumb;
      }
      else {
        return null;
      }
    } else {
      return null;
    }

    
  }

  public Pose2d getOdometry() {
    return swerve.getRobotPose();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     
    poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    
    Pose2d result = getVision();

    if (result != null) {
      poseEstimator.addVisionMeasurement(result, visionTimestamp);
    }
  }

  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }
}
