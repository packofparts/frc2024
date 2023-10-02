// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable table;
  private final String limelightName;

  private boolean _hasTargets;
  private double _latency;
  private double[] botPose;

  public Limelight(String name) {
    limelightName = name;
    table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  public void updateValues() {
    _hasTargets = table.getEntry("tv").getDouble(0) == 1;

    // Here we always use botpose_wpilib blue to establish a constant coordinate system that works
    // similarly to PathPlanner and Photonvision, where the bottom of blue is considered (0, 0)

    // Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
    botPose = table
                .getEntry("botpose_wpiblue")
                .getDoubleArray(new double[7]);
    
    _latency = botPose[6];
  }

  public boolean hasTargets() {
    return _hasTargets;
  }

  public Pose2d getVisionEstimatedPose(){
    double botX = botPose[0];
    double botY = botPose[1];
    double yaw = botPose[5];


    return new Pose2d(
      new Translation2d(botX, botY),
      Rotation2d.fromDegrees(yaw)
    );
  }
  public double getTimestamp() {
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(_latency);
  }


  @Override
  public void periodic() {
    updateValues();
  }
}
