// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightLime;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimationLimelight extends PoseEstimationBase {
  /** Creates a new PoseEstimationLimelight. */
  LimelightLime lime;
  boolean firstVisMeasurement = true;
  int numVis = 0;
  public PoseEstimationLimelight(LimelightLime lime, SwerveSubsystem swerve) {
    super(swerve);
    this.lime = lime;
  }

  public void updateVision() {
    if (lime.hasTargets()) {
      Pose2d pose = lime.getVisionEstimatedPose();
      if (firstVisMeasurement) {
        poseEstimator.addVisionMeasurement(pose, lime.getLatency());
        numVis += 1;
        if (numVis > 30)
          firstVisMeasurement = false;
      }
      else {
        if (getValidEstimate(pose)) {
          poseEstimator.addVisionMeasurement(pose, lime.getLatency());
        }
      }
    }
  }
  
  public boolean getValidEstimate(Pose2d result){
    Pose2d reference = poseEstimator.getEstimatedPosition();
    return Math.abs(result.minus(reference).getTranslation().getNorm()) < VisionConstants.visionEstimateThresholdMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    
  }
}
