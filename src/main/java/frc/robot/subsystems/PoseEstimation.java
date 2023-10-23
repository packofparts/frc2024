// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConfig;
import frc.robot.constants.VisionConstants;



public class PoseEstimation extends SubsystemBase{
  private  Limelight _limelight;
  private  SwerveSubsystem _swerve;
  private  SwerveDrivePoseEstimator _poseEstimator;
  private  Field2d _field = new Field2d();

  private boolean _hasUpdated = false; 

  public PoseEstimation(Limelight lime, SwerveSubsystem swerveSubsystem) {
    _limelight = lime;
    _swerve = swerveSubsystem;
    

    _poseEstimator = new SwerveDrivePoseEstimator(SwerveConfig.swerveKinematics,
        _swerve.getRotation2d(),
        _swerve.getModulePositions(),
        _swerve.getRobotPose(),
        VisionConstants.kStateStdDevs,
        VisionConstants.kVisionMeasurementStdDevs);
    _field = new Field2d();
    SmartDashboard.putData("Field", _field);
  }
  
  @Override
  public void periodic() {
    _poseEstimator.update(_swerve.getRotation2d(), _swerve.getModulePositions());
    updateVision();

    Pose2d pose = _poseEstimator.getEstimatedPosition();
    _field.setRobotPose(pose);
    SmartDashboard.putData("Field", _field);

    SmartDashboard.putNumber("PoseEst X", pose.getX());
    SmartDashboard.putNumber("PoseEst Y", pose.getY());
    SmartDashboard.putNumber("PoseEst Rot", pose.getRotation().getDegrees());

  }

  public void updateVision() {
    Optional<EstimatedRobotPose> pose = _limelight.getEstimatedGlobalPose(_poseEstimator.getEstimatedPosition());
    if (pose.isPresent()) {
      EstimatedRobotPose camPose = pose.get();
      if (isValidPose(camPose)) {
        _poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                                            camPose.timestampSeconds);
        SmartDashboard.putBoolean("isUpdatingVision", true);
        _hasUpdated = true;
        return;
      }
    }
    SmartDashboard.putBoolean("hasUpdatedVision", _hasUpdated);
    SmartDashboard.putBoolean("isUpdatingVision", false);

  }

  public boolean isValidPose(EstimatedRobotPose pose) {
    List<PhotonTrackedTarget> targets = pose.targetsUsed;
    if (targets.size() == 1){
      return targets.get(0).getPoseAmbiguity() < VisionConstants.kSingleTagAmbiguityThreshold;
    }


    return true;
  }

  public Pose2d getRobotPose() {
    return _poseEstimator.getEstimatedPosition();
    
  }

  public void resetPose() {
    _poseEstimator.resetPosition(_swerve.getRotation2d(), _swerve.getModulePositions(), new Pose2d());
  }
  public void resetPose(Pose2d pose) {
    _poseEstimator.resetPosition(_swerve.getRotation2d(), _swerve.getModulePositions(), pose);
  }

}