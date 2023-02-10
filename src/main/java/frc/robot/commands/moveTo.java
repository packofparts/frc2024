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
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class moveTo extends CommandBase {
  /** Creates a new moveTo. */
  
  public Transform2d transform;
  public Rotation2d rotation;
  public PIDController transController;
  public PIDController angleController;
  public SwerveSubsystem swerve;
  public Pose2d initPose;


  public SwerveDriveKinematics m_kinematics;
  

  public double xPoint;
  public double yPoint;
  public double rotPoint;

  public moveTo(Transform2d transform, SwerveSubsystem swervesub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.transform = transform;

    transController = new PIDController(0.2, 0, 0);
    angleController = new PIDController(0.02, 0, 0);
    transController.setTolerance(0.1);
    angleController.setTolerance(0.2);
    swerve = swervesub;
    addRequirements(swerve);

    initPose = swerve.getRobotPose();

    xPoint = initPose.getX() + transform.getX();
    yPoint = initPose.getY() + transform.getY();
    rotPoint = initPose.getRotation().getRadians() + transform.getRotation().getRadians();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = swerve.getRobotPose();

    double xSpeed = transController.calculate(pose.getX(), xPoint);
    double ySpeed = transController.calculate(pose.getY(), yPoint);
    double rot = angleController.calculate(pose.getRotation().getRadians(), rotPoint);
    SmartDashboard.putNumber("xSpeedCalc", xSpeed);
    SmartDashboard.putNumber("ySpeedCalc", ySpeed);
    SmartDashboard.putNumber("rotSpeedCalc", rot);
    SmartDashboard.putNumber("rotErr", angleController.getPositionError());
    SmartDashboard.putNumber("rotSetpoint", rotPoint);



    double magnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    if(magnitude < Constants.kMaxSpeedMPS*.8){
      xSpeed /= magnitude;
      ySpeed /= magnitude;
      xSpeed *= Constants.kMaxSpeedMPS;
      ySpeed *= Constants.kMaxSpeedMPS;
    }

    swerve.setMotors(xSpeed, ySpeed, rot);

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Transform2d difference = initPose.plus(transform).minus(swerve.getRobotPose());
    if (transController.atSetpoint() && angleController.atSetpoint())
      return true;
    return false;
  }
}
