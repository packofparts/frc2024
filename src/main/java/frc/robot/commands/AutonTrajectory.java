// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonTrajectory extends CommandBase {
  
  public SwerveSubsystem swerve;
  public Pose2d initPose;
  public Transform2d transform,difference;
  public int desiredStateID;
  public double deadZone, desiredEndHeading;
  public ChassisSpeeds speeds;
  public moveTo move;
  public String curvePath = "paths/scurvetest.wpilib.json";
  
  PIDController velocityController, angleController;

  Trajectory testPath, path;

  Field2d field;

  double startTime = 0;


  public AutonTrajectory(SwerveSubsystem swerve, double desiredEndHeading) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(curvePath);
    try {
      path = TrajectoryUtil.fromPathweaverJson(trajPath);
    } catch (IOException e) {}

    initPose = swerve.getRobotPose();
    

    startTime = Timer.getFPGATimestamp();

    desiredStateID = 0;

    deadZone = Constants.deadZone;
    this.desiredEndHeading = 90;


    // TrajectoryConfig tc = new TrajectoryConfig(.5, .5);
    // this.path = TrajectoryGenerator.generateTrajectory(initPose, 
    // List.of(
    //   new Translation2d(.1, 0),
    //   new Translation2d(0, -.1),
    //   new Translation2d(-.1, 0)
    // ), new Pose2d(initPose.getX(), initPose.getY(), Rotation2d.fromDegrees(initPose.getRotation().getDegrees()+0)), tc);


    for(int i = 0; i < path.getStates().size(); i++){
      
      

      path.getStates().get(i).poseMeters = path.getStates().get(i).poseMeters.times(1.5);
      //path.getStates().set(i, new Pose2d(a.getX()*1, a.getY()*1, a.getRotation()));
    }

    

    transform = path.getStates().get(0).poseMeters.minus(initPose); //not used

    velocityController = new PIDController(.5, 0, 0);
    angleController = new PIDController(0.02, 0, 0);
  

    this.swerve.field.getObject("traj").setTrajectory(path);

    this.swerve.m_odometry.resetPosition(this.swerve.getRotation2d(), this.swerve.getModuleStates(), path.getStates().get(0).poseMeters);

    addRequirements(swerve);
  }

    public Pose2d getPose(int idx, double heading) {
        //Getting pose and applying transformation
        Pose2d desiredPose = path.getStates().get(idx).poseMeters;


        //double totalTime = path.getState().get(path.getStates().size()-1).timeSeconds;
        // Calculating wanted heading
        double changeRot;

        changeRot = this.desiredEndHeading * ((double) idx / path.getStates().size());

        return new Pose2d(desiredPose.getX(), desiredPose.getY(), new Rotation2d(Units.degreesToRadians(changeRot)));
    }

    public void getSpeeds() {
        desiredStateID ++;
        Pose2d currentPose = swerve.getRobotPose();
        Pose2d desiredPose = getPose(desiredStateID, currentPose.getRotation().getDegrees());        
    
        move = new moveTo(desiredPose.minus(currentPose), swerve);
        move.schedule();
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // void trackMethod1(){
  //   // if(move.isFinished()){
  //   //   getSpeeds();
  //   // }
    
  //   difference = this.path.getStates().get(this.desiredStateID).poseMeters.minus(this.swerve.getRobotPose());
  //   if(Math.abs(difference.getX()) < Constants.deadZone && Math.abs(difference.getY()) < Constants.deadZone){
  //     desiredStateID++;
  //     difference = this.path.getStates().get(this.desiredStateID).poseMeters.minus(this.swerve.getRobotPose());
  //   }
  //   SmartDashboard.putNumber("Size", this.path.getStates().get(0).poseMeters.getX());
  //   SmartDashboard.putNumber("Trajectory Desired State", desiredStateID);
  //   SmartDashboard.putNumber("differencePoseX", difference.getX());
  //   SmartDashboard.putNumber("differencePoseY", difference.getY());
  //   SmartDashboard.putNumber("currentPoseX", this.swerve.getRobotPose().getX());
  //   SmartDashboard.putNumber("currentPoseY", this.swerve.getRobotPose().getY());


  //   double xSpeed = difference.getX();
  //   double ySpeed = difference.getY();
  //   double magnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

  //   double rot = angleController.calculate(this.swerve.getRobotPose().getRotation().getRadians(), this.swerve.getRobotPose().getRotation().getRadians() + difference.getRotation().getRadians());

  //   xSpeed /= magnitude ;
  //   ySpeed /= magnitude ;

  //   xSpeed *= 1;
  //   ySpeed *= 1;


  //   this.swerve.setMotors(xSpeed, ySpeed, 0);
  // }
  void trackMethod2(){
      State desiredState = this.path.getStates().get(desiredStateID);
      
      
           
      double desiredVelocity = Math.abs(desiredState.velocityMetersPerSecond);
      //double deltaTime = trajectoryPath.get(idx).timeSeconds - trajectoryPath.get(idx-1).timeSeconds;
  
      //difference = this.path.getStates().get(this.desiredStateID).poseMeters.plus(transform.times(-1));
      difference = this.path.getStates().get(this.desiredStateID).poseMeters.minus(this.swerve.getRobotPose());
      SmartDashboard.putNumber("differenceX", difference.getX());
      SmartDashboard.putNumber("differenceY", difference.getY());
      // SmartDashboard.putNumber("differenceRotation", )

      if(Math.abs(difference.getX()) < Constants.deadZone && Math.abs(difference.getY()) < Constants.deadZone){
        desiredStateID++;
        difference = this.path.getStates().get(this.desiredStateID).poseMeters.minus(this.swerve.getRobotPose());
        //difference = this.path.getStates().get(this.desiredStateID).poseMeters.plus(transform.times(-1));
      }

      
      double magnitude = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));

      double xSpeed = difference.getX() / magnitude * desiredVelocity * 1.3;
      double ySpeed = difference.getY() / magnitude * desiredVelocity * 1.3;
      
      double rot = angleController.calculate(this.swerve.getRobotPose().getRotation().getDegrees(), this.path.getStates().get(desiredStateID).poseMeters.getRotation().getDegrees());

      double rotation = difference.getRotation().getRadians();

      ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot*magnitude, this.swerve.getRobotPose().getRotation());

      this.swerve.setMotors(xSpeed, ySpeed, rot);
  }
 
  void trackMethod3(){
      State desiredState = this.path.sample(Timer.getFPGATimestamp() - startTime);
      
      
           
      double desiredVelocity = Math.abs(desiredState.velocityMetersPerSecond);
      //double deltaTime = trajectoryPath.get(idx).timeSeconds - trajectoryPath.get(idx-1).timeSeconds;
  
      //difference = this.path.getStates().get(this.desiredStateID).poseMeters.plus(transform.times(-1));
      difference = this.path.sample(Timer.getFPGATimestamp() - startTime).poseMeters.minus(this.swerve.getRobotPose());
      SmartDashboard.putNumber("differenceX", difference.getX());
      SmartDashboard.putNumber("differenceY", difference.getY());
      // SmartDashboard.putNumber("differenceRotation", )

      if(Math.abs(difference.getX()) < Constants.deadZone && Math.abs(difference.getY()) < Constants.deadZone){
        desiredStateID++;
        difference = this.path.sample(Timer.getFPGATimestamp() - startTime).poseMeters.minus(this.swerve.getRobotPose());
        //difference = this.path.getStates().get(this.desiredStateID).poseMeters.plus(transform.times(-1));
      }

      
      double magnitude = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));

      double xSpeed = difference.getX() / magnitude * desiredVelocity * 1;
      double ySpeed = difference.getY() / magnitude * desiredVelocity * 1;
      
      double rot = angleController.calculate(this.swerve.getRobotPose().getRotation().getDegrees(), this.path.sample(Timer.getFPGATimestamp() - startTime).poseMeters.getRotation().getDegrees());

      double rotation = difference.getRotation().getRadians();

      ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.swerve.getRobotPose().getRotation());

      this.swerve.setMotors(xSpeed, ySpeed, rot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("desiredState", desiredStateID);
    SmartDashboard.putNumber("curHeading", this.swerve.getRobotPose().getRotation().getDegrees());
    SmartDashboard.putNumber("desiHeading", this.path.getStates().get(desiredStateID).poseMeters.getRotation().getDegrees());
    
    trackMethod3();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (desiredStateID>=path.getStates().size()-1 && Math.abs(this.path.getStates().get(this.path.getStates().size()-1).poseMeters.getRotation().getDegrees() - this.swerve.getRobotPose().getRotation().getDegrees())<5) {return true;}
    return false;
  }
}
