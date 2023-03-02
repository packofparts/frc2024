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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
    //Initializing Subsystems
    this.swerve = swerve;
    lime = limelight;
    // Getting Tag Layout
    try {
      layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("biggestbird.json"));
    } catch (IOException e) {
      e.printStackTrace();
    }
    // Initializing poseEstimator
    poseEstimator = new SwerveDrivePoseEstimator(swerve.m_kinematics,
       swerve.getRotation2d(),
       swerve.getModulePositions(),
       getOdometry(),
       VisionConstants.stateStdDevs,
       VisionConstants.visionMeasurementStdDevs);
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     
    poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    
    updateVision();
  }

  public void updateVision() {
    PhotonPipelineResult image = lime.getImg();
    double timestamp = image.getTimestampSeconds();
    PhotonTrackedTarget target = image.getBestTarget();

    if (target != null) {
      
      if (target.getPoseAmbiguity()<=.2) {
        Pose3d targetpose = layout.getTagPose(target.getFiducialId()).get();

        Transform3d transformation = target.getBestCameraToTarget().inverse();

        targetpose.plus(transformation);
        targetpose.plus(VisionConstants.robotToCam);

        poseEstimator.addVisionMeasurement(targetpose.toPose2d(), timestamp);
      }
      
    }

    
  }

  public Pose2d getOdometry() {
    return swerve.getRobotPose();
  }
  



  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }


}
