// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import frc.robot.subsystems.LimelightLime;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseEstimationLimelight extends PoseEstimationBase {
  /** Creates a new PoseEstimationLimelight. */
  LimelightLime lime;
  public PoseEstimationLimelight(LimelightLime lime, SwerveSubsystem swerve) {
    super(swerve);
    this.lime = lime;
  }

  public void updateVision() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    
  }
}
