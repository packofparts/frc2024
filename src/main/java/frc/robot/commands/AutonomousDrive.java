// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonomousDrive extends CommandBase {
  /** Creates a new AutonomousDrive. */
  SwerveSubsystem swerve;
  SwerveControllerCommand yeet;
  Boolean done = false;
  public AutonomousDrive(SwerveSubsystem s) {
//     // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = s;
    addRequirements(swerve);
    this.swerve.resetRobotPose();
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      5,
      5) .setKinematics(this.swerve.m_kinematics);

// 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(1, 5),
                  new Translation2d(1, -2)),
          new Pose2d(3, -1, Rotation2d.fromDegrees(90)),
          trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(0.01, 0, 0);
    PIDController yController = new PIDController(0.01, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
          0.07, 0, 0, new Constraints(999, 999));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    yeet = new SwerveControllerCommand(
      trajectory,
      this.swerve::getRobotPose,
      this.swerve.m_kinematics,
      xController,
      yController,
      thetaController,
      this.swerve::getRotation2d,
      this.swerve::setModuleStates,
      this.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    yeet.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (yeet.isFinished()){
      done=true;
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
