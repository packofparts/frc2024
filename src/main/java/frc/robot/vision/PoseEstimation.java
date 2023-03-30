// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimation extends PoseEstimationBase {
 Field2d field2;
 boolean firstVisMeasurement = true;
 int numVis = 0;
 LimelightPhoton lime;
  
  public PoseEstimation(LimelightPhoton limelight, SwerveSubsystem swerve) {
    super(swerve);
    this.lime = limelight;
    estimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, limelight.photonCamera, VisionConstants.robotToCam);

    field2 = new Field2d();
    SmartDashboard.putData("Field2", field2);
  }

  @Override
  public void updateVision() {
    estimator.setReferencePose(super.getPosition());
    SmartDashboard.putBoolean("FirstVisMeasurement", firstVisMeasurement);
    SmartDashboard.putNumber("numVis", numVis);
    Optional<EstimatedRobotPose> result = estimator.update();

    if (result.isPresent()) {

      EstimatedRobotPose unpacked = result.get();
      Pose2d pose = unpacked.estimatedPose.toPose2d();
      field2.setRobotPose(pose);
      if(firstVisMeasurement ){

        poseEstimator.addVisionMeasurement(pose, unpacked.timestampSeconds);
        numVis += 1;
        

        if (numVis>30)
          firstVisMeasurement = false;

      }
      else{
        SmartDashboard.putBoolean("ValidEstimate", this.getValidEstimate(pose));

        if(this.getValidEstimate(pose)){
          poseEstimator.addVisionMeasurement(pose, unpacked.timestampSeconds);
        }  
      }


    }
  }

  public boolean getValidEstimate(Pose2d result){
    Pose2d reference = estimator.getReferencePose().toPose2d();
    SmartDashboard.putNumber("Distance from Reference", result.minus(reference).getTranslation().getNorm());

    return Math.abs(result.minus(reference).getTranslation().getNorm()) < VisionConstants.visionEstimateThresholdMeters;
  }

  @Override
  public void periodic() {

    super.periodic();
  }


}
