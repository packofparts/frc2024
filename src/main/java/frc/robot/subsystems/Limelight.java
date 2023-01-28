// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.Pipeline;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.HashMap;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;

import java.io.IOException;
import java.util.ArrayList;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  //NetworkTable table = NetworkTableInstance.getDefault();
  PhotonCamera photonCamera;
  NetworkTableInstance net =  NetworkTableInstance.getDefault();
  NetworkTable lime = net.getTable("photonvision");
  PhotonPipelineResult img;
  public static enum Pipeline{
    TAG,
    REFLECTION,
    DRIVE
  }
  HashMap <String, Integer> pipelineVals = new HashMap<>();
  HashMap <String, Pose2d> fiducialHashMap = new HashMap<>();
  public AprilTagFieldLayout layout;
  

  public Limelight() {
    photonCamera = new PhotonCamera(net, "gloworm");
    pipelineVals.put("TAG", 0);
    pipelineVals.put("REFLECTION", 1);
    pipelineVals.put("DRIVE", 2);


    try {
      layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("biggestbird.json"));
    } catch (IOException e) {
      e.printStackTrace();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    img = photonCamera.getLatestResult();
  }

  public double getXoffset(){
    PhotonTrackedTarget targ = img.getBestTarget();
    return -targ.getYaw();
    
  }

  public double getForwardDistance(){
    PhotonTrackedTarget targ = img.getBestTarget();
    return PhotonUtils.calculateDistanceToTargetMeters(0.05, 0.05, 0, targ.getPitch());
  }

  public void setPipeline(Pipeline p){
    photonCamera.setPipelineIndex(pipelineVals.get(p.name()));;
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

  public Pose3d estimatePose() {
    PhotonTrackedTarget target = getBestTarget();
    Transform3d transform = target.getBestCameraToTarget().inverse();

    return null;

    
  }


}
