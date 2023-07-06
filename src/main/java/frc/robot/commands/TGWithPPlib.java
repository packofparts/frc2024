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
  
  public TGWithPPlib(SwerveSubsystem swerve, PathPlannerTrajectory traj, HashMap<String,Command> eventMap) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.eventMap = eventMap;
    this.traj = traj;
    
    addRequirements(this.swerve);
  }
  
  @Override
  public void initialize() {
  cmd = new SwerveAutoBuilder(this.swerve::getRobotPose, this.swerve::resetRobotPose, this.swerve.m_kinematics,
   new PIDConstants(1, 0, 0), //old .4
    new PIDConstants(2, 0, 0), //old .5
    this.swerve::setModuleStates, this.eventMap, true, this.swerve);
    
    
    finalCMD = cmd.fullAuto(traj);
    finalCMD.schedule();
  }

  public Pose2d getFlippedPose(){
    return this.swerve.getRobotPose().plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)));
  }

  // Called every time the schedulxer runs while the command is scheduled.
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
