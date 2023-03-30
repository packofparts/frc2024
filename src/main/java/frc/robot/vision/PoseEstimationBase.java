// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimationBase extends SubsystemBase {
    /** Creates a new PoseEstimation. */
    public AprilTagFieldLayout layout;
    public PhotonPoseEstimator estimator;
  
   // Transformation from robot to 
   SwerveSubsystem swerve;
   SwerveDrivePoseEstimator poseEstimator;
   Field2d field;
   


  /** Creates a new PoseEstimationBase. */
  public PoseEstimationBase(SwerveSubsystem swerve) {
    this.swerve = swerve;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    poseEstimator = new SwerveDrivePoseEstimator(swerve.m_kinematics,
       swerve.getRotation2d(),
       swerve.getModulePositions(),
       getOdometry(),
       VisionConstants.stateStdDevs,
       VisionConstants.visionMeasurementStdDevs);
    field = new Field2d();
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    poseEstimator.update(swerve.getRotation2d(), swerve.getModulePositions());
    updateVision();
    

    Pose2d pose = getPosition();
    field.setRobotPose(pose);

    
    
    SmartDashboard.putNumber("X Pose", pose.getX());
    SmartDashboard.putNumber("Y Pose", pose.getY());
    SmartDashboard.putNumber("Rot Pose", pose.getRotation().getDegrees());
    SmartDashboard.updateValues();
  }

  public void updateVision() {
  }

  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getOdometry() {
    return swerve.getRobotPose();
  }


}
