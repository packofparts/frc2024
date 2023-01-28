// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    transController = new PIDController(0.5, 0, 0);
    angleController = new PIDController(0.5, 0, 0);
    swerve = swervesub;
    

    initPose = swerve.getRobotPose().transformBy(transform);

    xPoint = initPose.getX();
    yPoint = initPose.getY();
    rotPoint = initPose.getRotation().getRadians();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = swerve.getRobotPose();

    double magnitude = Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(),2));

    double xSpeed = transController.calculate(pose.getX(), xPoint);
    double ySpeed = transController.calculate(pose.getY(), yPoint);
    double rot = angleController.calculate(pose.getRotation().getRadians(), rotPoint);

    swerve.setMotors(xSpeed*3, ySpeed*3, rot);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Transform2d difference = initPose.minus(swerve.getRobotPose());

    if (Math.abs(difference.getX())<Constants.deadZone && Math.abs(difference.getY())<Constants.deadZone && Math.abs(difference.getRotation().getRadians())<Constants.radDeadZone) {
      return true;
    }

    return false;
  }
}
