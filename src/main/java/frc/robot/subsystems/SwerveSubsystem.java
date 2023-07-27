// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConfig;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  public static enum DriveMode {
    AUTO, TELEOP
  }



  public SwerveSubsystem(SwerveConfig config) {
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
