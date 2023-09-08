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
  NetworkTable table;
  String limelightName;

  double tv;
  double tl;
  double cl;
  double[] botPose;

  public Limelight(String name) {
    this.limelightName = name;
    this.table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  public void updateValues() {
    tv = table.getEntry("tv").getDouble(0);
    tl = table.getEntry("tl").getDouble(0);
    cl = table.getEntry("cl").getDouble(0);
    botPose = table
                .getEntry("botpose_wpiblue")
                .getDoubleArray(new double[1]);
  
  }

  public boolean hasTargets() {
    return tv == 1.0;
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
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl)-Units.millisecondsToSeconds(cl);
  }


  @Override
  public void periodic() {
    updateValues();
  }
}
