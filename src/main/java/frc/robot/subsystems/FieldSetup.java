// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldSetup extends SubsystemBase {
  /** Creates a new FieldSetup. */
  Field2d field = new Field2d();
  SwerveSubsystem swervee;
  public FieldSetup(SwerveSubsystem swerve) {
    this.swervee = swerve;
    field.setRobotPose(this.swervee.getRobotPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
