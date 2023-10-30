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
  private  Limelight mLimelight;
  private  SwerveSubsystem mSwerve;
  private  SwerveDrivePoseEstimator mPoseEstimator;
  private  Field2d mField = new Field2d();

  private boolean mHasUpdated = false; 

  public PoseEstimation(Limelight lime, SwerveSubsystem swerve) {
    mLimelight = lime;
    mSwerve = swerve;

    mPoseEstimator = new SwerveDrivePoseEstimator(SwerveConfig.SWERVE_KINEMATICS,
        mSwerve.getRotation2d(),
        mSwerve.getModulePositions(),
        mSwerve.getRobotPose(),
        VisionConstants.kStateStdDevs,
        VisionConstants.kVisionMeasurementStdDevs);
    mField = new Field2d();
    SmartDashboard.putData("Field", mField);
  }
  
  @Override
  public void periodic() {
    mPoseEstimator.update(mSwerve.getRotation2d(), mSwerve.getModulePositions());
    updateVision();

    Pose2d pose = mPoseEstimator.getEstimatedPosition();
    mField.setRobotPose(pose);
    SmartDashboard.putData("Field", mField);

    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("PoseEst X", pose.getX());
      SmartDashboard.putNumber("PoseEst Y", pose.getY());
      SmartDashboard.putNumber("PoseEst Rot", pose.getRotation().getDegrees());
    }

  }

  public void updateVision() {
    Optional<EstimatedRobotPose> pose = mLimelight.getEstimatedGlobalPose(mPoseEstimator.getEstimatedPosition());
    if (pose.isPresent()) {
      EstimatedRobotPose camPose = pose.get();
      if (isValidPose(camPose)) {
        mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                                            camPose.timestampSeconds);
        SmartDashboard.putBoolean("isUpdatingVision", true);
        mHasUpdated = true;
        return;
      }
    }
    SmartDashboard.putBoolean("hasUpdatedVision", mHasUpdated);
    SmartDashboard.putBoolean("isUpdatingVision", false);

  }

  public boolean isValidPose(EstimatedRobotPose pose) {
    List<PhotonTrackedTarget> targets = pose.targetsUsed;
    if (targets.size() == 1){
      return targets.get(0).getPoseAmbiguity() < VisionConstants.SINGLE_TAG_AMBIGUITY_THRESH;
    }

    return true;
  }

  public Pose2d getRobotPose() {
    return mPoseEstimator.getEstimatedPosition();
    
  }

  public void resetPose() {
    mPoseEstimator.resetPosition(mSwerve.getRotation2d(), mSwerve.getModulePositions(), new Pose2d());
  }
  public void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(mSwerve.getRotation2d(), mSwerve.getModulePositions(), pose);
  }

}