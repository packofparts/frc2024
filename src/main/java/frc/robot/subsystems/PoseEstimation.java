// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PoseEstimation extends SubsystemBase {
  /** Creates a new PoseEstimation. */
  public AprilTagFieldLayout layout;
  public PhotonPoseEstimator estimator;

 // Transformation from robot to 
 SwerveSubsystem swerve;
 Limelight lime;
 SwerveDrivePoseEstimator poseEstimator;
 Field2d field;
 boolean firstVisMeasurement = false;
 
  public PoseEstimation(Limelight limelight, SwerveSubsystem swerve) {
    this.swerve = swerve;
    lime = limelight;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }


    estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, limelight.photonCamera, VisionConstants.robotToCam);

    poseEstimator = new SwerveDrivePoseEstimator(swerve.m_kinematics,
       swerve.getRotation2d(),
       swerve.getModulePositions(),
       getOdometry(),
       VisionConstants.stateStdDevs,
       VisionConstants.visionMeasurementStdDevs);
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void updateVision() {
    estimator.setReferencePose(getPosition());
    
    Optional<EstimatedRobotPose> result = estimator.update();

    if (result.isPresent()) {

      EstimatedRobotPose unpacked = result.get();
      Pose2d pose = unpacked.estimatedPose.toPose2d();
      if (Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2)) < VisionConstants.maxDistance) {
        if(!firstVisMeasurement){
          poseEstimator.addVisionMeasurement(pose, unpacked.timestampSeconds);
        }else{
          if(this.getValidEstimate()){
            poseEstimator.addVisionMeasurement(pose, unpacked.timestampSeconds);
          }  
        }


      }
      

    }
  }

  public boolean getValidEstimate(){
    Pose2d visionEstimate = this.getPosition();
    Pose2d odomEstimate = this.getOdometry();

    return visionEstimate.minus(odomEstimate).getTranslation().getNorm() < VisionConstants.visionEstimateThresholdMeters;
  }

  public Pose2d getOdometry() {
    return swerve.getRobotPose();
  }

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
