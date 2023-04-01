// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.PoseEstimationBase;
import frc.robot.vision.PoseEstimationLimelight;

import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
public class TGWithPPlib extends CommandBase {
  /** Creates a new TGWithPPlib. */
  SwerveSubsystem swerve;
  SwerveAutoBuilder cmd;
  Command finalCMD;
  PathPlannerTrajectory traj;
  HashMap<String,Command> eventMap;
  PoseEstimationBase pose;
  public TGWithPPlib(SwerveSubsystem swervee, PathPlannerTrajectory traj, HashMap<String,Command> eventMape) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pose = pose;
    this.swerve = swervee;
    this.eventMap = eventMap;
    this.traj = traj;
    addRequirements(this.swerve);
    this.swerve.resetRobotPose(new Pose2d());
    
  }
  
  @Override
  public void initialize() {
  cmd = new SwerveAutoBuilder(this.swerve::getRobotPose, this.swerve::resetRobotPose, this.swerve.m_kinematics,
   new PIDConstants(0.4, 0, 0),
    new PIDConstants(0.5, 0, 0),
    this.swerve::setModuleStates, this.eventMap, true, this.swerve);
    finalCMD = cmd.fullAuto(traj);
    finalCMD.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return finalCMD.isFinished();
  }
}
