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
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class moveTo extends CommandBase {
  /** Creates a new moveTo. */
  
  public Transform2d transform;
  public Rotation2d rotation;
  public PIDController yController, xController;
  public PIDController angleController;
  public SwerveSubsystem swerve;



  public SwerveDriveKinematics m_kinematics;
  

  public double xPoint;
  public double yPoint;
  public double rotPoint;


  /**
   * Constructor that moves a certain transform from the current position
   * @param transform
   * @param swervesub
   */
  public moveTo(Transform2d transform, SwerveSubsystem swervesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.transform = transform;

    yController = new PIDController(0.4, 0, 0);
    xController = new PIDController(0.4, 0, 0);
    angleController = new PIDController(1.4, 0, 0);

    yController.setTolerance(0.05);
    xController.setTolerance(0.05);

    angleController.setTolerance(0.001);
    swerve = swervesub;
    addRequirements(swerve);

    
    xPoint = this.swerve.getRobotPose().getX() + this.transform.getX();
    yPoint = this.swerve.getRobotPose().getY() + this.transform.getY();
    rotPoint = this.swerve.getRobotPose().getRotation().getRadians() + transform.getRotation().getRadians();
  }


  /**
   * Contructor that goes to the pose specified
   * @param pose
   * @param swervesub
   */
  public moveTo(Pose2d pose, SwerveSubsystem swervesub) {
    // Use addRequirements() here to declare subsystem dependencies.

    yController = new PIDController(0.4, 0, 0); 
    xController = new PIDController(0.4, 0, 0);
    angleController = new PIDController(1.4, .02, 0);

    xController.setTolerance(0.3);
    yController.setTolerance(0.3);
    angleController.setTolerance(0.005);

    swerve = swervesub;
    addRequirements(swerve);

    xPoint = pose.getX();
    yPoint = pose.getY();
    rotPoint = pose.getRotation().getRadians();
  }

  @Override
  public void initialize(){

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = this.swerve.getRobotPose();

    double ySpeed = yController.calculate(pose.getY(), yPoint);
    double xSpeed = xController.calculate(pose.getX(), xPoint);
    double rot = angleController.calculate(pose.getRotation().getRadians(), rotPoint);

    SmartDashboard.putNumber("xSpeedCalc", xSpeed);
    SmartDashboard.putNumber("ySpeedCalc", ySpeed);
    SmartDashboard.putNumber("rotSpeedCalc", rot);
    SmartDashboard.putNumber("rotErr", angleController.getPositionError());
    SmartDashboard.putNumber("rotSetpoint", rotPoint);
    SmartDashboard.putBoolean("isAtRotSetpoint", angleController.atSetpoint());




    double magnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


    xSpeed /= magnitude;
    ySpeed /= magnitude;
    xSpeed *= DriveConstants.kPhysicalMaxSpeedMPS;
    ySpeed *= DriveConstants.kPhysicalMaxSpeedMPS;
    

    // Setting to a little less than 90 degrees per second as minimum speed
    // Otherwise PID slows down too much when approaching angle
    // Could maybe be resolved by calculating good PID values
    // Maybe substantially increase kP and then increase kD to correct for the oscillation
    if(Math.abs(rot) < 1.5 && !angleController.atSetpoint()){
      rot = 1.5 * (rot/Math.abs(rot));
    }

    if((!xController.atSetpoint() || !yController.atSetpoint()))
      swerve.setMotors(xSpeed, ySpeed, rot, DriveMode.AUTO);
    else
      swerve.setMotors(0, 0, rot, DriveMode.AUTO);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Transform2d difference = initPose.plus(transform).minus(swerve.getRobotPose());
    if (xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint()){
      SmartDashboard.putBoolean("isAtRotSetpoint", angleController.atSetpoint());
      return true;
    }
      
    return false;
  }
}
