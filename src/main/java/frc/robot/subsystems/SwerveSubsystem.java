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
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConfig;
import frc.robot.SwerveModule;
import frc.robot.constants.CompConstants;
import frc.robot.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveDriveKinematics mKinematics;
  private final SwerveDriveOdometry mOdometry;

  private final AHRS mNavX;

  private final SwerveModule[] mModules;

  public SwerveSubsystem() {
    // Populating Instance Variables
    mKinematics = SwerveConfig.SWERVE_KINEMATICS;
    mNavX = new AHRS(Port.kMXP);
    mModules = SwerveConfig.SWERVE_MODULES;
    mOdometry = new SwerveDriveOdometry(mKinematics, getRotation2d(), getModulePositions());
    resetGyro();
    resetRobotPose(new Pose2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mOdometry.update(getRotation2d(), getModulePositions());
    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("FLPIDOutput", mModules[0].getPIDOutputRot());
      SmartDashboard.putNumber("FRPIDOutput", mModules[1].getPIDOutputRot());
      SmartDashboard.putNumber("BLPIDOutput", mModules[2].getPIDOutputRot());
      SmartDashboard.putNumber("BRPIDOutput", mModules[3].getPIDOutputRot());

      SmartDashboard.putNumber("XPos", mOdometry.getPoseMeters().getX());
      SmartDashboard.putNumber("YPos", mOdometry.getPoseMeters().getY());
      SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());

      for (int i = 0; i < mModules.length; i++) {
        SmartDashboard.putNumber("AppliedOutput" + i, mModules[i].getAppliedOutput());
        SmartDashboard.putNumber("DesiredStateAngleDeg" + i,
            mModules[i].getDesiredRadiansRot() / Math.PI * 180);
        SmartDashboard.putNumber("RotRelativePosDeg" + i,
            mModules[i].getRotRelativePosition() * 360);
        SmartDashboard.putNumber("AbsEncoderDeg" + i, mModules[i].getRotPosition() / Math.PI * 180);
        SmartDashboard.putNumber("SpeedMeters" + i, mModules[i].getTransVelocity());
        SmartDashboard.putNumber("PosMeters" + i, mModules[i].getTransPosition());
      }
    }
  }

  /**
   * Sets the current YAW heading as the 0'd heading
   */
  public void resetGyro() {
    mNavX.reset();
  }

  /**
   * Resets pose to origin, keeps heading from gyro, keeps current module positions
   */

  public void resetOdometry() {
    mOdometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
  }

  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0) CCW (with neg)
   * 
   * @return the degrees at which the gyro is at
   */
  public double getHeading() {
    return -mNavX.getAngle();
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
      mModules[i].setDesiredState(desiredStates[i]);
    }

  }

  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft,
   *         backright]
   * @see SwerveModulePosition
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[mModules.length];

    for (int i = 0; i < mModules.length; i++) {
      positions[i] = mModules[i].getModulePos();
    }

    return positions;

  }

  /**
   * 
   * @param vxMPS this is the forward velocity in meters/second
   * @param vyMPS this is the sideways velocity in meter/second (left is positive)
   * @param angleSpeedRADPS this is in radians/second counterclockwise
   * @param fieldOriented this is a boolean that determines if the robot is field oriented or not
   * 
   * @apiNote Keep in mind all of this is field relative so resetting the gyro midmatch will also
   *          reset these params
   */
  public void setChassisSpeed(double vxMPS, double vyMPS, double angleSpeedRADPS,
      boolean fieldOriented) {
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS, vyMPS, angleSpeedRADPS, getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(vxMPS, vyMPS, angleSpeedRADPS);
    }
    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);


    setModuleStates(moduleStates);

  }

  public void setChassisSpeed(double x, double y, double rot) {
    setChassisSpeed(x, y, rot, false);
  }

  /**
   * This method resets the pose of the robot to the desired robot pose
   * 
   * @param pose provide the new desired pose of the robot
   * @see Pose2d
   */
  public void resetRobotPose(Pose2d pose) {
    mOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }


  /**
   * @return provide the pose of the robot in meters
   */
  public Pose2d getRobotPose() {
    return mOdometry.getPoseMeters();
  }


}
