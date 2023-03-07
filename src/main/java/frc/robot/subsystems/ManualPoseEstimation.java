// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.swing.text.Utilities;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.Constants.VisionConstants;


public class ManualPoseEstimation extends PoseEstimation {
  /** Creates a new PoseEstimation. */
  public AprilTagFieldLayout layout;
  public PhotonPoseEstimator estimator;
  public Field2d field;
  public static enum Strategy {
    BEST,
    AVERAGEALL,
  }

 // Transformation from robot to 
 SwerveSubsystem swerve;
 Limelight lime;
 SwerveDrivePoseEstimator poseEstimator;
 double visionTimestamp;
 Strategy strategy;
 
  public ManualPoseEstimation(Limelight limelight, SwerveSubsystem swerve, Strategy strategy) {
    super(limelight, swerve);
    this.strategy = strategy;
    //Initializing Subsystems
    this.swerve = swerve;
    lime = limelight;
    // Getting Tag Layout
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
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
      
      field = new Field2d();
      SmartDashboard.putData("Field", field);
  }

  @Override
  public void updateVision() {
    PhotonPipelineResult image = lime.getImg();
    double timestamp = image.getTimestampSeconds();
    PhotonTrackedTarget target = image.getBestTarget();
    
    Pose2d targetpose = getPoseFromTarget(target);

    if (targetpose != null) {
      poseEstimator.addVisionMeasurement(targetpose, timestamp);
      SmartDashboard.putBoolean("Updating Vision", true);
    } else {
      SmartDashboard.putBoolean("Updating Vision ", false);
    }
  }

  public Pose2d getPoseFromTarget(PhotonTrackedTarget target) {
    if (target != null) {
      SmartDashboard.putNumber("Pose Ambiguity", target.getPoseAmbiguity());

      if (target.getPoseAmbiguity()<=.2) {
        Pose3d targetpose = layout.getTagPose(target.getFiducialId()).get();

        Transform3d transformation = target.getBestCameraToTarget().inverse();
        


        targetpose = targetpose.plus(transformation);
        targetpose = targetpose.plus(VisionConstants.robotToCam.inverse());

        SmartDashboard.putNumber("Transformation X", targetpose.getX());
        SmartDashboard.putNumber("Transformation Y", targetpose.getY());
        SmartDashboard.putNumber("Transformation Rot", targetpose.toPose2d().getRotation().getDegrees());

        if (Math.sqrt(Math.pow(targetpose.getX(), 2) + Math.pow(targetpose.getY(), 2))<VisionConstants.maxDistance) 
          return targetpose.toPose2d();
      }
      
    }
    return null;

  }

  @Override
  public Pose2d getOdometry() {
    return swerve.getRobotPose();
  }

  @Override
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    updateVision();


    Pose2d pose = getPosition();
    field.setRobotPose(pose);
    
    SmartDashboard.putNumber("X Pose", pose.getX());
    SmartDashboard.putNumber("Y Pose", pose.getY());
    SmartDashboard.putNumber("Rot Pose", pose.getRotation().getDegrees());
    SmartDashboard.updateValues();
  }



}
