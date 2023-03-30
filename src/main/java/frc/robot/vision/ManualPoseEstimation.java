// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;


public class ManualPoseEstimation extends PoseEstimationBase{
  /** Creates a new PoseEstimation. */

  public static enum Strategy {
    BEST,
    AVERAGEALL,
  }


 double visionTimestamp;
 Strategy strategy;
 LimelightPhoton lime;
 
  public ManualPoseEstimation(LimelightPhoton limelight, SwerveSubsystem swerve, Strategy strategy) {
    super(swerve);
    this.strategy = strategy;
    //Initializing Subsystems
    lime.setPipeline(1);
  }

  @Override
  public void updateVision() {
    PhotonPipelineResult image = lime.getImg();
    double timestamp = image.getTimestampSeconds();
    PhotonTrackedTarget target;
    Pose2d targetpose = null;
    if (image.hasTargets()) {
      target = image.getBestTarget();
      
      targetpose = getPoseFromTarget(target);
    }

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

        Optional<Pose3d> targetpose = layout.getTagPose(target.getFiducialId());

        if (targetpose.isPresent()) {
          return null;
        }

        Transform3d transformation = target.getBestCameraToTarget().inverse();
        


        Pose3d targetPoseFinal = targetpose.get().transformBy(transformation);
        targetPoseFinal = targetPoseFinal.transformBy(VisionConstants.robotToCam.inverse());

        SmartDashboard.putNumber("Transformation X", targetPoseFinal.getX());
        SmartDashboard.putNumber("Transformation Y", targetPoseFinal.getY());
        SmartDashboard.putNumber("Transformation Rot", targetPoseFinal.toPose2d().getRotation().getDegrees());

        //if (Util.getMagnitude(targetPoseFinal.getX(), targetPoseFinal.getY())<VisionConstants.maxDistance) 
          return targetPoseFinal.toPose2d();
      }
      
    }
    return null;

  }

  
  public Pose2d getOdometry() {
    return swerve.getRobotPose();
  }

  
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  
  @Override
  public void periodic() {

    super.periodic();
  }



}
