// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConfig;
import frc.robot.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private SwerveDriveKinematics _kinematics;
  private SwerveDriveOdometry _odometry;

  private AHRS _navx;

  private SwerveModule[] _modules;
  public static final double tuningOutput = 0;
  public static double autoGyroInitValue = 0;


  public SwerveSubsystem() {
    // Populating Instance Variables
    _kinematics = SwerveConfig.swerveKinematics;
    _navx = new AHRS(Port.kMXP);
    _modules = SwerveConfig.swerveModules;


    _odometry = new SwerveDriveOdometry(_kinematics, getRotation2d(), getModulePositions());
    resetGyro();
    resetRobotPose(new Pose2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _odometry.update(getRotation2d(), getModulePositions());
    if (SwerveConstants.kDebugMode){


      SmartDashboard.putNumber("FLPIDOutput", _modules[0].PIDOutput);
      SmartDashboard.putNumber("FRPIDOutput", _modules[1].PIDOutput);
      SmartDashboard.putNumber("BLPIDOutput", _modules[2].PIDOutput);
      SmartDashboard.putNumber("BRPIDOutput", _modules[3].PIDOutput);

      SmartDashboard.putNumber("XPos", _odometry.getPoseMeters().getX());
      SmartDashboard.putNumber("YPos", _odometry.getPoseMeters().getY());
      SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());



    }

    for (int i = 0; i < _modules.length; i++){
      SmartDashboard.putNumber("AppliedOutput"+i, _modules[i].getAppliedOutput());
      SmartDashboard.putNumber("DesiredStateAngle"+i, _modules[i].desiredRadians/Math.PI*180);
      SmartDashboard.putNumber("RotRelativePosDeg"+i, _modules[i].getRotRelativePosition()*360);
      SmartDashboard.putNumber("AbsEncoderDeg"+i, _modules[i].getRotPosition()/Math.PI*180);
      SmartDashboard.putNumber("SpeedMeters"+i, _modules[i].getTransVelocity());
      SmartDashboard.putNumber("PosMeters"+i, _modules[i].getTransPosition());
    }


    if (Input.resetGyro()){
      resetGyro();
    }
    if(Input.resetOdo()){
      _odometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
    }
  }

  /**
   * Sets the current YAW heading as the 0'd heading
   */
  public void resetGyro() {
    _navx.reset();
  }

  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0) CCW (with neg)
   * 
   * @return the degrees at which the gyro is at
   */
  public double getHeading() {
    return -_navx.getAngle();
  }

  /**
   * This gets the Rotation2d of the gyro (which is in continuous input)
   * 
   * @return the Rotation2d of the gyro CCW POSITIVE(Unit Circle Rise UP)
   * @see Rotation2d
   */
  public Rotation2d getRotation2d() {

    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * This function sets the current speeds of the swerve modules to the following array pattern
   * [frontleft, frontright, backleft, backright]
   * 
   * @see SwerveModuleState
   * @param desiredStates requires a SwerveModuleState array
   */

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kTeleMaxSpeedMPS);

    for (int i = 0; i < desiredStates.length; i++) {
      _modules[i].setDesiredState(desiredStates[i]);
    }

  }

  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft,
   *         backright]
   * @see SwerveModulePosition
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[_modules.length];

    for (int i = 0; i < _modules.length; i++) {
      positions[i] = _modules[i].getModulePos();
    }

    return positions;

  }

  /**
   * 
   * @param x this is the forward velocity in meters/second
   * @param y this is the sideways velocity in meter/second (left is positive)
   * @param rot this is in radians/second counterclockwise
   * @param fieldOriented this is a boolean that determines if the robot is field oriented or not
   * 
   * @apiNote Keep in mind all of this is field relative so resetting the gyro midmatch will also
   *          reset these params
   */
  public void setMotors(double x, double y, double rot, boolean fieldOriented) {
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(x, y, rot);
    }
    SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(chassisSpeeds);


    this.setModuleStates(moduleStates);

  }

  public void setMotors(double x, double y, double rot) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x, y, rot);
    SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(chassisSpeeds);


    this.setModuleStates(moduleStates);
  }

  /**
   * This method resets the pose of the robot to the desired robot pose
   * 
   * @param pose provide the new desired pose of the robot
   * @see Pose2d
   */
  public void resetRobotPose(Pose2d pose) {
    _odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }


  /**
   * @return provide the pose of the robot in meters
   */
  public Pose2d getRobotPose() {
    return _odometry.getPoseMeters();
  }


}
