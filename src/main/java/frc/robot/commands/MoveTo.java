// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;
import frc.robot.vision.PoseEstimation;

public class MoveTo extends CommandBase {
  /** Creates a new moveTo. */
  
  public Transform2d transform;
  public Rotation2d rotation;
  public PIDController yController, xController;
  public PIDController angleController;
  public SwerveSubsystem swerve;
  public PoseEstimation estimator;


  public SwerveDriveKinematics m_kinematics;
  

  public double xPoint;
  public double yPoint;
  public double rotPoint;

  private boolean usePoseEstimator = false;
  private Pose2d currentPose;

  /**
   * Constructor that moves a certain transform from the current position
   * @param transform
   * @param swervesub
   */
  public MoveTo(Transform2d transform, SwerveSubsystem swervesub, PoseEstimation estimator) {
    // Use addRequirements() here to declare subsystem dependencies.
    usePoseEstimator = true;
    
    this.transform = transform;
    this.estimator = estimator;
    swerve = swervesub;
    addRequirements(swerve);

    yController = PIDConstants.YController;
    xController = PIDConstants.XController;
    angleController = PIDConstants.rotController;

    yController.setTolerance(0.05);
    xController.setTolerance(0.05);

    angleController.setTolerance(0.001);


    
    xPoint = estimator.getPosition().getX() + this.transform.getX();
    yPoint = estimator.getPosition().getY() + this.transform.getY();
    rotPoint = estimator.getPosition().getRotation().getRadians() + transform.getRotation().getRadians();
  }

  public MoveTo(Pose2d desiredPose, SwerveSubsystem swervesub, PoseEstimation estimator) {
    // Use addRequirements() here to declare subsystem dependencies.
    usePoseEstimator = true;
    
    this.estimator = estimator;
    swerve = swervesub;
    addRequirements(swerve);

    yController = PIDConstants.YController;
    xController = PIDConstants.XController;
    angleController = PIDConstants.rotController;

    yController.setTolerance(0.05);
    xController.setTolerance(0.05);


    angleController.setTolerance(0.001);

    
    xPoint = desiredPose.getX();
    yPoint = desiredPose.getY();
    rotPoint = desiredPose.getRotation().getRadians();
  }


  // /**
  //  * Contructor that goes to the pose specified
  //  * @param pose
  //  * @param swervesub
  //  */
  // public MoveTo(Pose2d pose, SwerveSubsystem swervesub) {
  //   // Use addRequirements() here to declare subsystem dependencies.
    
    
  //   yController = new PIDController(0.4, 0, 0); 
  //   xController = new PIDController(0.4, 0, 0);
  //   angleController = new PIDController(1.8, .02, .75);

  //   xController.setTolerance(0.1);
  //   yController.setTolerance(0.2);
  //   angleController.setTolerance(0.02);

  //   swerve = swervesub;
  //   addRequirements(swerve);

  //   xPoint = pose.getX();
  //   yPoint = pose.getY();
  //   rotPoint = pose.getRotation().getRadians();
  // }

  public MoveTo(Transform2d transform, SwerveSubsystem swervesub){
    swerve = swervesub;
    addRequirements(swerve);

    yController = new PIDController(PIDConstants.transPIDValues[0], PIDConstants.transPIDValues[1], PIDConstants.transPIDValues[2]);
    xController = new PIDController(PIDConstants.transPIDValues[0], PIDConstants.transPIDValues[1], PIDConstants.transPIDValues[2]);
    angleController = new PIDController(PIDConstants.rotPIDValues[0], PIDConstants.rotPIDValues[1], PIDConstants.rotPIDValues[2]);

    yController.setTolerance(0.1);
    xController.setTolerance(0.1);
    angleController.setTolerance(0.02);

    this.transform = transform;
    xPoint = this.swerve.getRobotPose().getX() + this.transform.getX();
    yPoint = this.swerve.getRobotPose().getY() + this.transform.getY();
    rotPoint = this.swerve.getRobotPose().getRotation().getRadians() + this.transform.getRotation().getRadians();
  }

  @Override
  public void initialize(){

    if (usePoseEstimator) {
      xPoint = estimator.getPosition().getX() + this.transform.getX();
      yPoint = estimator.getPosition().getY() + this.transform.getY();
      rotPoint = estimator.getPosition().getRotation().getRadians() + transform.getRotation().getRadians();
    }
    else {
      xPoint = this.swerve.getRobotPose().getX() + this.transform.getX();
      yPoint = this.swerve.getRobotPose().getY() + this.transform.getY();
      rotPoint = this.swerve.getRobotPose().getRotation().getRadians() + this.transform.getRotation().getRadians();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(this.usePoseEstimator){
      currentPose = estimator.getPosition();
    }else{
      currentPose = this.swerve.getRobotPose();
    }
    

    double ySpeed = yController.calculate(currentPose.getY(), yPoint);
    double xSpeed = xController.calculate(currentPose.getX(), xPoint);
    double rot = angleController.calculate(currentPose.getRotation().getRadians(), rotPoint);

    
    SmartDashboard.putNumber("xSpeedCalc", xSpeed);
    SmartDashboard.putNumber("ySpeedCalc", ySpeed);
    SmartDashboard.putNumber("rotSpeedCalc", rot);
    SmartDashboard.putNumber("rotErr", angleController.getPositionError());
    SmartDashboard.putNumber("rotSetpoint", rotPoint);
    SmartDashboard.putBoolean("isAtRotSetpoint", angleController.atSetpoint());


    double magnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


    
    
    double fx = (xSpeed / magnitude) * (DriveConstants.kPhysicalMaxSpeedMPS/3);
    double fy = (ySpeed / magnitude) * (DriveConstants.kPhysicalMaxSpeedMPS/3);
    

    // Setting to a little less than 90 degrees per second as minimum speed
    // Otherwise PID slows down too much when approaching angle
    // Could maybe be resolved by calculating good PID values
    // Maybe substantially increase kP and then increase kD to correct for the oscillation
    if(Math.abs(rot) < .7 && !angleController.atSetpoint()){
      rot = .7 * (rot/Math.abs(rot));
    }

    if((!xController.atSetpoint() || !yController.atSetpoint()))
      swerve.setMotors(xSpeed + fx, ySpeed + fy, rot, DriveMode.AUTO);
    else
      swerve.setMotors(0, 0, rot, DriveMode.AUTO);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopAllAndBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Transform2d difference = initPose.plus(transform).minus(swerve.getRobotPose());
    if (xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint()){
      SmartDashboard.putBoolean("IsMoveTo", false);
      SmartDashboard.putBoolean("isAtRotSetpoint", angleController.atSetpoint());
      return true;
    
    
    }
      
    return false;
  }
}
