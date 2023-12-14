// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.decomposition.eig.WatchedDoubleStepQRDecomposition_DDRM;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class WCDBclass extends SubsystemBase {
  private final DifferentialDrive mDifferentialDrive;
  private final DifferentialDriveKinematics mKinematics;
  private final DifferentialDriveOdometry mOdometry;
  // This is the current gyro on the westcoast drivebase
  private final Pigeon2 mGyro;
  // Define all CANSparkMax objects here
  private final wcModule leftModule;
  private final wcModule rightModule;

  /** Creates a new WCDBclass. */
  public WCDBclass(wcModule leftModule, wcModule rightModule) {
    mGyro = new Pigeon2(0);
    mKinematics = new DifferentialDriveKinematics(0); // Remember to plug in the distance between
                                                      // the wheels
    this.leftModule = leftModule;
    this.rightModule = rightModule;

    // plug in left and right spark maxs here
    mDifferentialDrive = new DifferentialDrive(leftModule.getMotor(), rightModule.getMotor());
    mOdometry = new DifferentialDriveOdometry(getRotation(), leftModule.getDistance(),
        rightModule.getDistance());
  }

  public void setCurvatureDrive() {
    // Use the DifferentialDrive class's curavturedrive method to control the overall movement of
    // the robot in TELEOP
    mDifferentialDrive.curvatureDrive(0.1, 0, false);
  }

  public void setArcadeDrive() {
    // Use the DifferentialDrive class's arcadeDrive method to control the overall movement of the
    // robot in TELEOP
    mDifferentialDrive.arcadeDrive(0.1, 0, false);
  }

  public void setTankDrive() {
    // Use the DifferentialDrive class's arcadeDrive method to control the overall movement of the
    // robot in TELEOP
    mDifferentialDrive.tankDrive(0.1, 0, false);
  }

  // The setChassisSpeed and setWheelSpeeds commnds are pretty much universal in every drivebase
  // here is the documentation on how it works:
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html

  public DifferentialDriveWheelSpeeds setChassisSpeed(ChassisSpeeds speed) {
    // Use the differential drive kinematics class to convert Chassis Speeds into wheel speeds
    return mKinematics.toWheelSpeeds(speed);
  }

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // pass on the array of wheel velocities to each individual WCModule Which should handle the
    // speeds with a PID controller
    leftModule.setVelocity(speeds.leftMetersPerSecond);
    rightModule.setVelocity(speeds.rightMetersPerSecond);
  }

  // This getPose method is found in almost every drivebase
  // here is the documentaion on how it works:
  // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html
  public void getPose() {
    // Use the Differential Drive Odometry class to get the current relative pose of the robot
    mOdometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // Update the odometry object with motor and gyro states
    mOdometry.update(getRotation(), leftModule.getDistance(), rightModule.getDistance());
  }

  private Rotation2d getRotation() {
    return (new Rotation2d(Math.toRadians(mGyro.getYaw())));
  }
}
