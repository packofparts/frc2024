// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.Pipeline;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.HashMap;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.ArrayList;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  //NetworkTable table = NetworkTableInstance.getDefault();
  PhotonCamera photonCamera;
  NetworkTableInstance net =  NetworkTableInstance.getDefault();
  NetworkTable lime;
  public PhotonPipelineResult img;
  public static enum Pipeline{
    TAG,
    REFLECTION,
    DRIVE,
    CUBE
  }
  HashMap <String, Integer> pipelineVals = new HashMap<>();
  HashMap <String, Pose2d> fiducialHashMap = new HashMap<>();
  public AprilTagFieldLayout layout;
  public PhotonPoseEstimator estimator;

  // Transformation from robot to 
  public final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  

  public Limelight() {
    lime = net.getTable("photonvision");

    photonCamera = new PhotonCamera(net, "OV5647");
    pipelineVals.put("TAG", 1);
    pipelineVals.put("REFLECTION", 2);
    pipelineVals.put("DRIVE", 3);
    pipelineVals.put("CUBE", 0);

                

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    img = photonCamera.getLatestResult();
    if (img.getBestTarget() != null) {
      SmartDashboard.putNumber("LimelightYaw", img.getBestTarget().getYaw());
    }
  }

  public double getSkew(){
    PhotonTrackedTarget targ = img.getBestTarget();
    //check positive direction
    return targ.getSkew();
    
  }

  public double getForwardDistance(double targetHeightMeters){
    PhotonTrackedTarget targ = img.getBestTarget();
    return PhotonUtils.calculateDistanceToTargetMeters(0.05, targetHeightMeters, 0, targ.getPitch());
  }

  public void setPipeline(int PipelineIndex){
    photonCamera.setPipelineIndex(PipelineIndex);
  }

public void addAprilTag(HashMap<String,Object>[]Tags){
    for (HashMap<String,Object> object : Tags) {
      fiducialHashMap.put((String)object.get("ID"),(Pose2d)object.get("POSE"));
    }
  }


  public Object hasTarg(Supplier<Object> Func){
    if (img.hasTargets()){
      return Func.get();
    }
    return null;
  }

  public PhotonTrackedTarget getBestTarget() {
    return img.getBestTarget();
  }
  public double getTimestamp() {
    return img.getTimestampSeconds();
  }
  public PhotonPipelineResult getImg() {
    return photonCamera.getLatestResult();
  }






}
