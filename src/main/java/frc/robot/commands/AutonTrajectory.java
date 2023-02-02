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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonTrajectory extends CommandBase {
  /** Creates a new Trajectory. */

;
  public Trajectory path = new Trajectory();
  public SwerveSubsystem swerve;
  public Pose2d initPose;
  public Transform2d transform, difference;
  public int idx;
  public double deadZone, desiredEndHeading;
  public ChassisSpeeds speeds;
  public MoveTo move;
  public String curvePath = "paths/sussybakacurve.wpilib.json";
  
  PIDController velocityController, angleController;

  Trajectory testPath;



  public AutonTrajectory(SwerveSubsystem swerve, double desiredEndHeading) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(curvePath);
    try {
      path = TrajectoryUtil.fromPathweaverJson(trajPath);
    } catch (IOException e) {}

    
    idx = 0;

    deadZone = Constants.deadZone;
    this.desiredEndHeading = desiredEndHeading;


    TrajectoryConfig tc = new TrajectoryConfig(1, .5);
    path = TrajectoryGenerator.generateTrajectory(initPose, 
    List.of(
      new Translation2d(1, 0),
      new Translation2d(0, -1),
      new Translation2d(-1, 0)
    ), new Pose2d(initPose.getX(), initPose.getY(), Rotation2d.fromDegrees(initPose.getRotation().getDegrees()+180)), tc);

    for(int i = 0; i < path.getStates().size(); i++){
      Transform2d a = path.getStates().get(i).poseMeters.minus(path.getStates().get(0).poseMeters);
      path.getStates().get(i).poseMeters = new Pose2d(a.getX(), a.getY(), a.getRotation());
    }

    initPose = swerve.getRobotPose();
    transform = path.getInitialPose().minus(initPose); //not used

    velocityController = new PIDController(.5, 0, 0);
    angleController = new PIDController(0.2, 0, 0);
    

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
        idx ++;
        Pose2d currentPose = swerve.getRobotPose();
        Pose2d desiredPose = getPose(idx, currentPose.getRotation().getDegrees());        
    
        move = new MoveTo(desiredPose.minus(currentPose), swerve);
        move.schedule();
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(move.isFinished()){
    //   getSpeeds();
    // }
    
    difference = this.path.getStates().get(this.idx).poseMeters.minus(this.swerve.getRobotPose());
    if(Math.abs(difference.getX()) < Constants.deadZone && Math.abs(difference.getY()) < Constants.deadZone){
      idx++;
      difference = this.path.getStates().get(this.idx).poseMeters.minus(this.swerve.getRobotPose());
    }

    SmartDashboard.putNumber("Trajectory Desired State", idx);
    SmartDashboard.putNumber("differencePoseX", difference.getX());
    SmartDashboard.putNumber("differencePoseY", difference.getY());
    SmartDashboard.putNumber("currentPoseX", this.swerve.getRobotPose().getX());
    SmartDashboard.putNumber("currentPoseY", this.swerve.getRobotPose().getY());

    double xSpeed = difference.getX();
    double ySpeed = difference.getY();
    double magnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    double rot = angleController.calculate(this.swerve.getRobotPose().getRotation().getRadians(), this.swerve.getRobotPose().getRotation().getRadians() + difference.getRotation().getRadians());

    xSpeed /= magnitude * .35;
    ySpeed /= magnitude * .35;


    this.swerve.setMotors(xSpeed, ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (idx>=path.getStates().size()-1) {return true;}
    return false;
  }
}
