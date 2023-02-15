// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
public class TGWithPPlib extends CommandBase {
  /** Creates a new TGWithPPlib. */
  SwerveSubsystem swerve;
  SwerveAutoBuilder cmd;
  Command finalCMD;
  PathPlannerTrajectory traj = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3));
  public TGWithPPlib(SwerveSubsystem swervee) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swervee;
    addRequirements(this.swerve);
    this.swerve.resetRobotPose(new Pose2d());
    
  }
  
  @Override
  public void initialize() {
    
  cmd = new SwerveAutoBuilder(this.swerve::getRobotPose, this.swerve::resetRobotPose,this.swerve.m_kinematics,
   new PIDConstants(0.5, 0, 0),
    new PIDConstants(0.5, 0, 0),
    this.swerve::setModuleStates, FieldConstants.eventMap, true, this.swerve);
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
