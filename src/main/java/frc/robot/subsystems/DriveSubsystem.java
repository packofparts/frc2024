// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.SwervePath;

public class DriveSubsystem extends SubsystemBase {
  
  public static double translationP, translationI, translationD, rotationP, rotationI, rotationD;


  public boolean isFieldOriented = true;


  private SwerveModule frontLeft = new SwerveModule(0, 1, true, true, "frontLeft");
  private SwerveModule frontRight = new SwerveModule(2, 3, true, true, "frontRight");
  private SwerveModule backLeft = new SwerveModule(4, 5, true, true, "backLeft");
  private SwerveModule backRight = new SwerveModule(6, 7, true, true, "backRight");
  

  private double kMaxSpeed = 10;
  private double chassisRotationStickMultiplier = 1;


  private Field2d  m_field = new Field2d();
  private SwerveDriveOdometry m_odometry;
  private SwerveDriveKinematics m_kinematics;

  

  private SimDouble angle;
  private int dev;
  double desiredRobotRot, simYaw = 0;

  int desiredStateIndex = 0;

  Joysticks joyee;

  double xPositionForTesting = 0;

  SwerveModule[] swerveModules = {
    frontLeft, 
    frontRight,
    backLeft,
    backRight
  };

  SwerveModulePosition[] swerveModulePositions = {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition(),
    
  };


  AHRS navx = new AHRS(Port.kMXP);


  String sCurveJSON = "paths/SCurve.wpilib.json";
  Trajectory scurvepath = new Trajectory();

  SwervePath path;
  public static Trajectory squarePath;

  public DriveSubsystem() {
    joyee = new Joysticks();
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2)
    );

    dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d(), swerveModulePositions);

    Pose2d initialPose = new Pose2d(Robot.path.getInitialPose().getX(), Robot.path.getInitialPose().getY(), getRotation2d());

    m_odometry.resetPosition(getRotation2d(), swerveModulePositions, initialPose);
    m_odometry.update(getRotation2d(), swerveModulePositions);
    
    
    putPIDValues();

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("initial pose x", initialPose.getX());
    SmartDashboard.putNumber("initial pose y", initialPose.getY());


    Pose2d initPose = initialPose;

    TrajectoryConfig tc = new TrajectoryConfig(1, .5);
    squarePath = TrajectoryGenerator.generateTrajectory(initPose, 
    List.of(
      new Translation2d(8, 4),
      new Translation2d(8, 0),
      new Translation2d(12, 0)
    ), new Pose2d(initPose.getX(), initPose.getY(), Rotation2d.fromDegrees(initPose.getRotation().getDegrees()+0)), tc);


    m_field.getObject("traj").setTrajectory(squarePath);

    path = new SwervePath(getHeading(), 90, 0.25);
    
  }


  @Override
  public void simulationPeriodic() {

    // updatePIDValues();

    // double x = -joyee.getX();
    // double y = -joyee.getY();
    // double rot = joyee.getRot();

    // x = Math.abs(x) > 0.15 ? x : 0.0;
    // y = Math.abs(y) > 0.15 ? y : 0.0;
    // rot = Math.abs(rot) > 0.15 ? rot : 0.0;


    // SmartDashboard.putNumber("TransStickX", x);
    // SmartDashboard.putNumber("odometryX", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("odometryY", m_odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("odometryRot", m_odometry.getPoseMeters().getRotation().getDegrees());

    // desiredRobotRot += rot * chassisRotationStickMultiplier; 


    // // if(checkIfReachedDesiredState){
    // //   desiredStateIndex++;
    // // }
    
    // //this.setMotors(x, y, rot);

    // checkIfReachedDesiredState();

    swerveModulePositions[0] = frontLeft.getPosition();
    swerveModulePositions[1] = frontRight.getPosition();
    swerveModulePositions[2] = backLeft.getPosition();
    swerveModulePositions[3] = backRight.getPosition();

    SwerveModuleState[] moduleStates = {
      swerveModules[0].getState(),
      swerveModules[1].getState(),
      swerveModules[2].getState(),
      swerveModules[3].getState()
    };

    ChassisSpeeds chassisSpeed = m_kinematics.toChassisSpeeds(moduleStates);
    double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;


    simYaw += chassisRotationSpeed * 0.02;

    
    angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-Units.radiansToDegrees(simYaw));

    
    
    m_odometry.update(getRotation2d(), swerveModulePositions);
    
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void resetGyro(){
    navx.reset();
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getHeading(){
    return Math.IEEEremainder(-navx.getAngle(), 360); 

  }

  public void setMotors(double x, double y, double rot){
    x *= -kMaxSpeed; y *= kMaxSpeed;
    rot *= chassisRotationStickMultiplier;

    ChassisSpeeds chassisSpeeds;

    if(isFieldOriented)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    else
      chassisSpeeds = new ChassisSpeeds(x, y, rot);

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    this.setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);


    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }


  public void putPIDValues(){
    SmartDashboard.putNumber("translationP", 0);
    SmartDashboard.putNumber("translationI", 0);
    SmartDashboard.putNumber("translationD", 0);
    SmartDashboard.putNumber("rotationP", 0);
    SmartDashboard.putNumber("rotationI", 0);
    SmartDashboard.putNumber("rotationD", 0);
  }

  public void updatePIDValues(){
      translationP = SmartDashboard.getNumber("translationP", 0);
      translationI = SmartDashboard.getNumber("translationI", 0);
      translationD = SmartDashboard.getNumber("translationD", 0);
      rotationP = SmartDashboard.getNumber("rotationP", 0);
      rotationI = SmartDashboard.getNumber("rotationI", 0);
      rotationD = SmartDashboard.getNumber("rotationD", 0);
  }

  void checkIfReachedDesiredState(){

    ChassisSpeeds desiredSpeeds = path.getSpeeds(m_odometry.getPoseMeters());

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);
    
    
    //setAllModuleStates(desiredState, -desiredRotation);
    this.setModuleStates(moduleStates);
  }

  void setAllModuleStates(State desiredState, double desiredRotation){


    frontLeft.setDesiredState(desiredState, Rotation2d.fromDegrees(desiredRotation));
    frontRight.setDesiredState(desiredState, Rotation2d.fromDegrees(desiredRotation));
    backLeft.setDesiredState(desiredState, Rotation2d.fromDegrees(desiredRotation));
    backRight.setDesiredState(desiredState, Rotation2d.fromDegrees(desiredRotation));

  }

  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }
  
}
