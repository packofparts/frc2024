// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightLime extends SubsystemBase {
  /** Creates a new LimelightLime. */
  NetworkTable LimelightTable;
  String limelightName;
  double tv;
  double ta;
  double tl;


  public LimelightLime(String limelightName) {
      this.limelightName = limelightName;

      LimelightTable = NetworkTableInstance.getDefault().getTable(limelightName);


  }

  public void updateValues() {

    //TODO: Maybe switch these to use the limelight helper sub
    tv = LimelightTable.getEntry("tv").getDouble(0);
    ta = LimelightTable.getEntry("ta").getDouble(0);
    tl = LimelightTable.getEntry("tl").getDouble(40);
  }

  public boolean hasTargets() {
    return tv == 1;
  
  }
  
  public Pose2d getVisionEstimatedPose() {

    double[] bot_pose;

    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        bot_pose = LimelightTable
                .getEntry("botpose_wpiblue")
                .getDoubleArray(new double[1]);
    } else {
        bot_pose = LimelightTable
                .getEntry("botpose_wpired")
                .getDoubleArray(new double[1]);
    }

    double bot_x = bot_pose[0];
    double bot_y = bot_pose[1];
    double rotation_z = (bot_pose[5] + 360) % 360;


    return new Pose2d(
            new Translation2d(bot_x, bot_y),
            Rotation2d.fromDegrees(rotation_z));
  }
  public double getLatency() {
    return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl);
 
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
