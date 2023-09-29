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
  private SwerveSubsystem _swerve;
  private Limelight _lime;

  // Fields
  private Field2d field = new Field2d();

  // Estimator
  private SwerveDrivePoseEstimator _poseEstimator;
  public PoseEstimation(SwerveSubsystem swerve, Limelight limelight) {
    _swerve = swerve;
    _lime = limelight;

    _poseEstimator = new SwerveDrivePoseEstimator(SwerveConfig.swerveKinematics,
       swerve.getRotation2d(),
       swerve.getModulePositions(),
       swerve.getRobotPose(),
       VisionConstants.kStateStdDevs,
       VisionConstants.kVisionMeasurementStdDevs);

  }



  @Override
  public void periodic() {
    // Updating pose estimator
    SmartDashboard.putData("Field", field);

    _poseEstimator.update(_swerve.getRotation2d(), _swerve.getModulePositions());
    updateVision();

    // Updating Field
    field.setRobotPose(getPosition());  
  }

  public Pose2d getPosition() {
    return _poseEstimator.getEstimatedPosition();
  }
  
  public void updateVision() {
    if (_lime.hasTargets()) {
      Pose2d pose = _lime.getVisionEstimatedPose();
      if (pose.getY() < 4) {
        double time = _lime.getTimestamp();
        _poseEstimator.addVisionMeasurement(pose, time);
      }

    }
  }

}