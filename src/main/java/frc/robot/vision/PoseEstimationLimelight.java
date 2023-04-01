// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightLime;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimationLimelight extends PoseEstimationBase {
  /** Creates a new PoseEstimationLimelight. */
  LimelightLime lime;
  boolean firstVisMeasurement = true;
  int numVis = 0;
  Field2d field3;
  public PoseEstimationLimelight(LimelightLime lime, SwerveSubsystem swerve) {
    super(swerve);
    this.lime = lime;
    field3 = new Field2d();
    SmartDashboard.putData("Field3", field3);
  }

  @Override
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
          SmartDashboard.putBoolean("Updated Vision", true);
        } else {
          SmartDashboard.putBoolean("Updated Vision", false);
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
    poseEstimator.update(SwerveSubsystem.getRotation2d(), swerve.getModulePositions());
    updateVision();
    

    Pose2d pose = getPosition();
    field.setRobotPose(pose);

    
    
    SmartDashboard.putNumber("X Pose", pose.getX());
    SmartDashboard.putNumber("Y Pose", pose.getY());
    SmartDashboard.putNumber("Rot Pose", pose.getRotation().getDegrees());
    SmartDashboard.updateValues();

    field3.setRobotPose(getPosition());
    
    
  }
}
