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
import frc.robot.constants.CompConstants;
import frc.robot.constants.VisionConstants;



public class PoseEstimation extends SubsystemBase{
  private  Limelight limelight;
  private  SwerveSubsystem swerve;
  private  SwerveDrivePoseEstimator poseEstimator;
  private  Field2d field = new Field2d();

  private boolean hasUpdated = false; 

  public PoseEstimation(Limelight lime, SwerveSubsystem swerveSubsystem) {
    limelight = lime;
    swerve = swerveSubsystem;
    

    poseEstimator = new SwerveDrivePoseEstimator(SwerveConfig.SWERVE_KINEMATICS,
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
    poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    updateVision();

    Pose2d pose = poseEstimator.getEstimatedPosition();
    field.setRobotPose(pose);
    SmartDashboard.putData("Field", field);

    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("PoseEst X", pose.getX());
      SmartDashboard.putNumber("PoseEst Y", pose.getY());
      SmartDashboard.putNumber("PoseEst Rot", pose.getRotation().getDegrees());
    }

  }

  public void updateVision() {
    Optional<EstimatedRobotPose> pose = limelight.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    if (pose.isPresent()) {
      EstimatedRobotPose camPose = pose.get();
      if (isValidPose(camPose)) {
        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                                            camPose.timestampSeconds);
        SmartDashboard.putBoolean("isUpdatingVision", true);
        hasUpdated = true;
        return;
      }
    }
    SmartDashboard.putBoolean("hasUpdatedVision", hasUpdated);
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
    return poseEstimator.getEstimatedPosition();
    
  }

  public void resetPose() {
    poseEstimator.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), new Pose2d());
  }
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), pose);
  }

}