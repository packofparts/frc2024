// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConfig;
import frc.robot.constants.VisionConstants;

public class PoseEstimation extends SubsystemBase {
  
  // Subsystems
  SwerveSubsystem swerve;
  Limelight lime;

  // Fields
  AprilTagFieldLayout layout;
  Field2d field;

  // Estimator
  SwerveDrivePoseEstimator poseEstimator;
  public PoseEstimation(SwerveSubsystem swerve, Limelight limelight) {
    this.swerve = swerve;
    this.lime = limelight;

    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    poseEstimator = new SwerveDrivePoseEstimator(SwerveConfig.swerveKinematics,
       swerve.getRotation2d(),
       swerve.getModulePositions(),
       swerve.getRobotPose(),
       VisionConstants.kStateStdDevs,
       VisionConstants.kVisionMeasurementStdDevs);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    
  }

  @Override
  public void periodic() {
    // Updating pose estimator
    poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    updateVision();

    // Updating Field
    field.setRobotPose(getPosition());  
  }

  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }
  
  public void updateVision() {
    if (this.lime.hasTargets()) {
      Pose2d pose = this.lime.getVisionEstimatedPose();
      double time = this.lime.getTimestamp();
      poseEstimator.addVisionMeasurement(pose, time);

    }
  }

}
