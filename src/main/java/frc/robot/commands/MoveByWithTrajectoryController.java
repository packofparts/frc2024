// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveByWithTrajectoryController extends CommandBase {
  /** Creates a new MoveByWithTarjectoryController. */
  SwerveSubsystem swervee;
  SwerveControllerCommand yeet;
  Transform2d trans;
  public MoveByWithTrajectoryController(SwerveSubsystem swerve,Transform2d translation) {
    this.swervee = swerve;
    addRequirements(this.swervee);
    this.trans = translation;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putNumber("Xtrans", this.trans.getTranslation().times(1.0/2).getX());
    SmartDashboard.putNumber("Ytrans", this.trans.getTranslation().times(1.0/2).getY());
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      1,
      1).setKinematics(this.swervee.m_kinematics);
    
    

  // 2. Generate trajectory
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //       this.swervee.getRobotPose(),
    //       List.of(
    //               this.trans.getTranslation().times(1.0/4)
    //       ),
    //       new Pose2d(this.trans.getTranslation().times(1.0/2), this.swervee.getRobotPose().getRotation()),
    //       trajectoryConfig);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
          new Pose2d(this.swervee.getRobotPose().getTranslation(), this.swervee.getRotation2d()),
          new Pose2d(0, 0, new Rotation2d())
      ),
      trajectoryConfig
    );

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(
          0.1, 0, 0, new Constraints(5, 3));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    yeet = new SwerveControllerCommand(
      trajectory,
      this.swervee::getRobotPose,
      this.swervee.m_kinematics,
      xController,
      yController,
      thetaController,
      this.swervee::setModuleStates,
      this.swervee);
      yeet.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yeet.isFinished();
  }
}
