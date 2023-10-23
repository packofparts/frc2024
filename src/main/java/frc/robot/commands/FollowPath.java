package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveConfig;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;


import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class FollowPath extends CommandBase {
  /** Creates a new TGWithPPlib. */
  SwerveSubsystem swerve;
  SwerveAutoBuilder cmd;
  Command finalCMD;
  PathPlannerTrajectory traj;
  HashMap<String,Command> eventMap;
  PoseEstimation pose;
  
  public FollowPath(SwerveSubsystem swerve, PathPlannerTrajectory traj, HashMap<String,Command> eventMap, PoseEstimation pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.eventMap = eventMap;
    this.traj = traj;
    this.pose = pose;
    
    addRequirements(this.swerve);

  }
  @Override
  public void initialize() {

    cmd = new SwerveAutoBuilder(this.pose::getRobotPose, this.pose::resetPose, SwerveConfig.swerveKinematics,
            new PIDConstants(7, 0.5, 0), //old .4
            new PIDConstants(9, 0.5, 0), //old .5
            this.swerve::setModuleStates, this.eventMap, true, this.swerve
        );
    
    finalCMD = cmd.fullAuto(traj);

    SwerveSubsystem.autoGyroInitValue = traj.getInitialHolonomicPose().getRotation().getDegrees();

    finalCMD.schedule();
    SmartDashboard.putBoolean("pathFinished", finalCMD.isFinished());
    SmartDashboard.putBoolean("pathSchedules", finalCMD.isScheduled());
  }
  @Override
  public void execute(){
    SmartDashboard.updateValues();
  }


  // Called every time the schedulxer runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // For now we are keeping this empty to handle interruptions
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return finalCMD.isFinished();
  }
}