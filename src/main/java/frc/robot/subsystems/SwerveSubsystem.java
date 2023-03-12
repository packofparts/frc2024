/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI.Port;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  // Bevel gears must be facing to the left in order to work
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveCANId, 
    DriveConstants.kFrontLeftSteerCANId,
    0 /* abs encoder analog channel number */,
    false /* driving motor inversion */, 
    false /* steering motor inversion */,
    0.389 /* abs encoder offset */,
    false /* abs encoder inversion */, 
    true /* is using abs encoder */,
    PIDConstants.kFrontLeftSteeringPIDControl, 
    PIDConstants.kFrontLeftDrivingMotorController);

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveCANId, 
    DriveConstants.kFrontRightSteerCANId,
    1 /* abs encoder analog channel number */,
    true /* driving motor inversion */,
    false /* steering motor inversion */,
    0.321 /* abs encoder offset */,
    false /* abs encoder inversion */,
    true /* is using abs encoder */,
    PIDConstants.kFrontRightSteeringPIDControl,
    PIDConstants.kFrontRightDrivingMotorController);

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveCANId, 
    DriveConstants.kBackLeftSteerCANId,
    2 /* abs encoder analog channel number */,
    false /* driving motor inversion */,
    false /* steering motor inversion */,
    0.751 /* abs encoder offset */,
    false /* abs encoder inversion */, 
    true /* is using abs encoder */,
    PIDConstants.kBackRightSteeringPIDControl,
    PIDConstants.kFrontLeftDrivingMotorController);

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveCANId, 
    DriveConstants.kBackRightSteerCANId,
    3 /* abs encoder analog channel number */,
    true /* driving motor inversion */,
    false /* steering motor inversion */,
    0.546 /* abs encoder offset */,
    false /* abs encoder inversion */, 
    true /* is using abs encoder */,
    PIDConstants.kBackLeftSteeringPIDControl,
    PIDConstants.kBackRightDrivingMotorController); 

  private final PIDController headingController;

  public SwerveDriveKinematics m_kinematics;
  private ChassisSpeeds chassisSpeeds1;
  public SwerveDriveOdometry m_odometry;
  AHRS navx = new AHRS(Port.kMXP);
  SwerveModule [] rawMods;
  
  public static enum DriveMode{
    AUTO,
    TELEOP
  }

  public SwerveSubsystem() {
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidthMeters / 2),
      new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidthMeters / 2),
      new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidthMeters / 2),
      new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidthMeters / 2));
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d(), this.getModulePositions());
    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
    resetGyro();

    headingController = new PIDController(0.5, 0, 0);

    resetRobotPose(new Pose2d());
    rawMods = getRawModules();
    setIdleModeForAll(IdleMode.kBrake, IdleMode.kBrake);
    headingController.setTolerance(Units.degreesToRadians(5));
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Roll", getRoll());

    for(int i = 0; i<getRawModules().length;i++){
      SmartDashboard.putNumber("RelativeEnc"+i, getRawModules()[i].getRotPosition());
      SmartDashboard.putNumber("TruePos"+i, getRawModules()[i]._universalEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Generic"+i, getRawModules()[i]._universalEncoder.getAbsolutePosition()-getRawModules()[i]._universalEncoder.getPositionOffset());
      SmartDashboard.putNumber("Offset"+i, getRawModules()[i]._universalEncoder.getPositionOffset());
      SmartDashboard.putNumber("TransEncoderPos"+i, getRawModules()[i].getTransPosition()/10);
      SmartDashboard.putNumber("TransEncoderVelocity"+i, getRawModules()[i].getTransVelocity());
    }

    m_odometry.update(getRotation2d(), getModulePositions());
    SmartDashboard.putNumber("OdometryX", getRobotPose().getX());
    SmartDashboard.putNumber("OdometryY", getRobotPose().getY());
    SmartDashboard.putNumber("RotationDegrees", getRobotPose().getRotation().getDegrees());
    if(Input.resetGyro()){resetGyro();}
    if(Input.resetPose()){
      this.resetRobotPose(new Pose2d());
    }
  }
  /**
   * Sets the current YAW heading as the 0'd heading
   */
  public void resetGyro(){
    navx.reset();
  }

  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0) CCW (with neg)
   * @return the degrees at which the gyro is at
   */
  public double getHeading(){
    SmartDashboard.putNumber("Gyro Heading",-navx.getAngle());
    return -navx.getAngle();
  }

  /**
   * This gets the Rotation2d of the gyro (which is in continuous input)
   * @return the Rotation2d of the gyro
   * @see Rotation2d
   */
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public Rotation2d geRotation2dNotCCW(){
    return Rotation2d.fromDegrees(-getHeading());
  }

    /**
   * This function sets the current speeds of the swerve modules to the following array pattern
   * [frontleft, frontright, backleft, backright]
   * @see SwerveModuleState
   * @param desiredStates requires a SwerveModuleState array
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, DriveMode mode) {
    switch(mode){
      case AUTO:
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kAutoMaxSpeedMPS);
        break;
      
      case TELEOP:
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleMaxSpeedMPS);
        break;
      }
      
      frontLeft.setDesiredState(desiredStates[0],mode);
      frontRight.setDesiredState(desiredStates[1],mode);
      backLeft.setDesiredState(desiredStates[2],mode);
      backRight.setDesiredState(desiredStates[3],mode);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kAutoMaxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0],DriveMode.AUTO);
    frontRight.setDesiredState(desiredStates[1],DriveMode.AUTO);
    backLeft.setDesiredState(desiredStates[2],DriveMode.AUTO);
    backRight.setDesiredState(desiredStates[3],DriveMode.AUTO);
  }

  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft, backright]
   * @see SwerveModulePosition
   */
  public SwerveModulePosition[] getModulePositions(){
    return(new SwerveModulePosition[]{frontLeft.getModulePos(),frontRight.getModulePos(),backLeft.getModulePos(),backRight.getModulePos()});
  }

  /**
   * 
   * @param x this is the forward velocity in meters/second
   * @param y this is the sideways velocity in meter/second (left is positive)
   * @param rot this is in radians/second counterclockwise
   * 
   * @apiNote Keep in mind all of this is field relative so resetting the gyro midmatch will also reset these params
   */
  public void setMotors(double x,double y, double rot, DriveMode dMode){
    if (!Input.getRobotOriented()){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x, y, rot);}
    SmartDashboard.putNumber("ChassispeedsX",chassisSpeeds1.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassispeedsy",chassisSpeeds1.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassispeedsRadians",chassisSpeeds1.omegaRadiansPerSecond);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);


    this.setModuleStates(moduleStates, dMode);
  }

  public void setMotors(double x,double y, double rot){
    if (!Input.getRobotOriented()){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x,y, rot);}
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);
    this.setModuleStates(moduleStates, DriveMode.AUTO);
  }

  /**
   * This method resets the pose of the robot to the desired robot pose
   * @param pose provide the new desired pose of the robot
   * @see Pose2d
   */
  public void resetRobotPose(Pose2d pose){
    m_odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  /**
   * @return provide the pose of the robot in meters
   */
  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }

  /**
   * makes all modules return to its original state
   */
  public void goToOrigin(){
    System.out.println("executed");
    for (SwerveModule mod : rawMods){
      mod.returnToOrigin();
    }
  }

  /**
   * @return an array of all the SwerveModule objects in the format [frontLeft,frontRight,backLeft,backRight]
   */
  public SwerveModule[] getRawModules(){
    return new SwerveModule[]{frontLeft,frontRight,backLeft,backRight};
  }

  /**
   * stops all swerve module rotation and translation
   */
  public void stopAllAndBrake(){
    for (SwerveModule mod : rawMods){
      mod.stop();
    }
  }

  /**
   * sets the Idle mode for all the modules
   * @param transMode idle mode for translation motor
   * @param rotMode idle mode for rotation motor
   * @see IdleMode
   */
  public void setIdleModeForAll(IdleMode transMode,IdleMode rotMode){
    for (SwerveModule mod : rawMods){
      mod.setModeRot(rotMode);
      mod.setModeTrans(transMode);
      mod.applySettings();
      mod.burnSparks();
    }
  }

  /**
   * forms an X shape with the wheels for grip
   */
  public void setXShape(){
    setModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(0,new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45)))
    });
  }

  /**
   * 
   * @return returns roll in degrees (180,-180)
   */
  public float getRoll(){
    return this.navx.getRoll(); 
  }

  /**
   * 
   * @return returns yaw in degrees (180,-180)
   */
  public float getYaw(){
    return this.navx.getYaw();
  }

  /**
   * 
   * @return returns pitch in degrees (180,-180)
   */
  public float getPitch(){
    return this.navx.getPitch();
  }
}