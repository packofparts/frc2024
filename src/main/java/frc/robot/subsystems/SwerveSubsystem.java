/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



//Bababbooey


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CompConstants;
import frc.robot.Constants .DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import java.io.*;
import java.util.*;
import org.json.simple.*;
import org.json.simple.parser.*;

public class SwerveSubsystem extends SubsystemBase {

  // Bevel gears must be facing to the left in order to work

  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveCANId, DriveConstants.kFrontLeftSteerCANId,
   0,false, false,DriveConstants.kFrontLeftOffset,false, CompConstants.useAbsEncoder,
   PIDConstants.kFrontLeftSteeringPIDControl, PIDConstants.kFrontLeftSteeringPIDControl);

   private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightDriveCANId, DriveConstants.kFrontRightSteerCANId,
   1,true,false,DriveConstants.kFrontRightOffset,false, CompConstants.useAbsEncoder,
   PIDConstants.kFrontRightSteeringPIDControl,PIDConstants.kFrontLeftDrivingMotorController);

  private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftDriveCANId, DriveConstants.kBackLeftSteerCANId,
  2,false,false,DriveConstants.kBackLeftOffset,false, CompConstants.useAbsEncoder,
  PIDConstants.kBackLeftSteeringPIDControl,PIDConstants.kBackLeftSteeringPIDControl);

  public static double autoGyroInitValue = 0;

//above 0.5
  private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightDriveCANId, DriveConstants.kBackRightSteerCANId,
  3,true,false,DriveConstants.kBackRightOffset,false, CompConstants.useAbsEncoder,
   PIDConstants.kBackRightSteeringPIDControl,PIDConstants.kBackRightSteeringPIDControl); 



  private final PIDController headingController;

  public SwerveDriveKinematics m_kinematics;
  private ChassisSpeeds chassisSpeeds1;
  public SwerveDriveOdometry m_odometry;
  static AHRS navx = new AHRS(Port.kMXP);
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

    resetGyro();

    headingController = new PIDController(0.5, 0, 0);

    resetRobotPose(new Pose2d());
    rawMods = getRawModules();
    setIdleModeForAll(IdleMode.kBrake, IdleMode.kBrake);
    // backRight.setModeRot(IdleMode.kCoast);
    // backRight.setModeTrans(IdleMode.kCoast);
    headingController.setTolerance(Units.degreesToRadians(5));
    //rawMods[0].setModeTrans(IdleMode.kCoast);
    //rawMods[0].burnSparks();
  }

  @Override
  public void periodic() {
    if(DriveConstants.isChild){
      //DriveConstants.kTeleDriveMaxAccMPS *= DriveConstants.childFactor;
      //DriveConstants.kTeleDriveMaxAngularAccRadPS *= DriveConstants.childFactor;
      DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond = DriveConstants.childFactor;
      DriveConstants.kTeleMaxSpeedMPS = 1.15;
    } else{
      DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
      DriveConstants.kTeleMaxSpeedMPS = 4;

    }
    if (Input.getChild()){
      DriveConstants.isChild = !DriveConstants.isChild;
    }
    if (CompConstants.debug) {
      SmartDashboard.putNumber("Pitch", getPitch());
      SmartDashboard.putNumber("Yaw", getYaw());
      SmartDashboard.putNumber("Roll", getRoll());
      for(int i = 0; i<getRawModules().length;i++){
        SmartDashboard.putNumber("RelativeEnc"+i, getRawModules()[i].getRotPosition());
        SmartDashboard.putNumber("TruePos"+i, getRawModules()[i]._analogEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Generic"+i, getRawModules()[i]._analogEncoder.getAbsolutePosition()-getRawModules()[i]._analogEncoder.getPositionOffset());
        SmartDashboard.putNumber("Offset"+i, getRawModules()[i]._analogEncoder.getPositionOffset());
        SmartDashboard.putNumber("TransEncoderPos"+i, getRawModules()[i].getTransPosition());
        SmartDashboard.putNumber("TransEncoderVelocity"+i, getRawModules()[i].getTransVelocity()*DriveConstants.driveEncoderConversionFactortoRotations);
      }
    }

    m_odometry.update(getRotation2d(), getModulePositions());
    SmartDashboard.putNumber("OdometryX", getRobotPose().getX());
    SmartDashboard.putNumber("OdometryY", getRobotPose().getY());
    SmartDashboard.putNumber("RotationDegrees", getRobotPose().getRotation().getDegrees());

    if(Input.resetGyro()){resetGyro();}

  }
  /**
   * Sets the current YAW heading as the 0'd heading
   */
  public static void resetGyro(){
    navx.reset();
  }


  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0) CCW (with neg)
   * @return the degrees at which the gyro is at
   */
  public static double getHeading(){
    SmartDashboard.putNumber("Gyro Heading",-navx.getAngle());
    return -navx.getAngle();
  }



  /**
   * This gets the Rotation2d of the gyro (which is in continuous input)
   * @return the Rotation2d of the gyro
   * @see Rotation2d
   */
  public static Rotation2d getRotation2d(){
    if(DriverStation.isAutonomous()){
      return Rotation2d.fromDegrees(getHeading() + SwerveSubsystem.autoGyroInitValue);
    }else{
      return Rotation2d.fromDegrees(getHeading());
    }
    
  }

  public static Rotation2d geRotation2dNotCCW(){
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
  public void setMotors(double x,double y, double rot, DriveMode dMode, boolean fieldOriented){
    if (CompConstants.kGyroHold){
      rot = headingController.calculate(Units.degreesToRadians(navx.getRate()), rot);
    }

    if (fieldOriented){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x, y, rot);}
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);


    this.setModuleStates(moduleStates, dMode);
  }
  public void setMotors(double x,double y, double rot, DriveMode dMode){
    if (CompConstants.kGyroHold){
      rot = headingController.calculate(Units.degreesToRadians(navx.getRate()), rot);
    }
    if (!Input.getRobotOriented()){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x, y, rot);}
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);


    this.setModuleStates(moduleStates, dMode);
  }

  public void setMotors(double x,double y, double rot){
    if (CompConstants.kGyroHold){
      rot = headingController.calculate(Units.degreesToRadians(navx.getRate()), rot);
    }
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

  public double optimize(double rad) {
    return Math.IEEEremainder(Math.abs(rad), 2*Math.PI)*Math.signum(rad);

    // return (Math.abs(rad) %  (2*Math.PI)) * Math.signum(rad);
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
  public static float getRoll(){
    return SwerveSubsystem.navx.getRoll(); 
  }

  /**
   * 
   * @return returns yaw in degrees (180,-180)
   */
  public static float getYaw(){
    return SwerveSubsystem.navx.getYaw();
  }

  /**
   * 
   * @return returns pitch in degrees (180,-180)
   */
  public static float getPitch(){
    return SwerveSubsystem.navx.getPitch();
  }

  public void updateAbsEncOffsets(){
    JSONParser parser = new JSONParser();

    try {
      Object json1 = parser.parse(new FileReader(new File("src\\main\\deploy\\AbsolutEncoder.json")));
      JSONObject json = (JSONObject)json1;

      DriveConstants.kFrontLeftOffset = (double)json.get("frontLeft");
      DriveConstants.kFrontRightOffset = (double)json.get("frontRight");
      DriveConstants.kBackLeftOffset = (double)json.get("backLeft");
      DriveConstants.kBackRightOffset = (double)json.get("backRight");

      // this.getRawModules()[0].setOffset(DriveConstants.kFrontLeftOffset);
      // this.getRawModules()[1].setOffset(DriveConstants.kFrontRightOffset);
      // this.getRawModules()[2].setOffset(DriveConstants.kBackLeftOffset);
      // this.getRawModules()[3].setOffset(DriveConstants.kBackRightOffset);

      ArmConstants.pivotInitOffset = (double)json.get("pivot");
      

    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}