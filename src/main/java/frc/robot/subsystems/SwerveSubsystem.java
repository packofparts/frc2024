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
import frc.robot.SwerveModule;
import frc.robot.constants.CompConstants;
import frc.robot.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final AHRS navx;

  private final SwerveModule[] modules;
  public static double autoGyroInitValue = 0;


  public SwerveSubsystem() {
    // Populating Instance Variables
    kinematics = SwerveConfig.SWERVE_KINEMATICS;
    navx = new AHRS(Port.kMXP);
    modules = SwerveConfig.SWERVE_MODULES;


    odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions());
    resetGyro();
    resetRobotPose(new Pose2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation2d(), getModulePositions());
    if (CompConstants.DEBUG_MODE){


      SmartDashboard.putNumber("FLPIDOutput", modules[0].PIDOutput);
      SmartDashboard.putNumber("FRPIDOutput", modules[1].PIDOutput);
      SmartDashboard.putNumber("BLPIDOutput", modules[2].PIDOutput);
      SmartDashboard.putNumber("BRPIDOutput", modules[3].PIDOutput);

      SmartDashboard.putNumber("XPos", odometry.getPoseMeters().getX());
      SmartDashboard.putNumber("YPos", odometry.getPoseMeters().getY());
      SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());



    }

    for (int i = 0; i < modules.length; i++){
      SmartDashboard.putNumber("AppliedOutput"+i, modules[i].getAppliedOutput());
      SmartDashboard.putNumber("DesiredStateAngleDeg"+i, modules[i].desiredRadians/Math.PI*180);
      SmartDashboard.putNumber("RotRelativePosDeg"+i, modules[i].getRotRelativePosition()*360);
      SmartDashboard.putNumber("AbsEncoderDeg"+i, modules[i].getRotPosition()/Math.PI*180);
      SmartDashboard.putNumber("SpeedMeters"+i, modules[i].getTransVelocity());
      SmartDashboard.putNumber("PosMeters"+i, modules[i].getTransPosition());
    }



  }

  /**
   * Sets the current YAW heading as the 0'd heading
   */
  public void resetGyro() {
    navx.reset();
  }
  
  /**
   * Resets pose to origin, keeps heading from gyro, keeps current module positions
   */

  public void resetOdometry(){
    odometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
  }

  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0) CCW (with neg)
   * 
   * @return the degrees at which the gyro is at
   */
  public double getHeading() {
    return -navx.getAngle();
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.TELE_MAX_SPEED_MPS);

    for (int i = 0; i < desiredStates.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }

  }

  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft,
   *         backright]
   * @see SwerveModulePosition
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getModulePos();
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
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);


    this.setModuleStates(moduleStates);

  }

  public void setMotors(double x, double y, double rot) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x, y, rot);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);


    this.setModuleStates(moduleStates);
  }

  /**
   * This method resets the pose of the robot to the desired robot pose
   * 
   * @param pose provide the new desired pose of the robot
   * @see Pose2d
   */
  public void resetRobotPose(Pose2d pose) {
    odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }


  /**
   * @return provide the pose of the robot in meters
   */
  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }


}
