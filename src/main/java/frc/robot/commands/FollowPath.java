package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveConfig;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class FollowPath extends CommandBase {
  /** Creates a new TGWithPPlib. */
  private final SwerveSubsystem swerve;
  private final Command finalCMD;
  private final PathPlannerTrajectory traj;
  private final Map<String,Command> eventMap;
  private final PoseEstimation pose;
  
  public FollowPath(SwerveSubsystem tmpSwerve, PathPlannerTrajectory tmpTraj, Map<String,Command> tmpEventMap, PoseEstimation tmpPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = tmpSwerve;
    eventMap = tmpEventMap;
    traj = tmpTraj;
    pose = tmpPose;
    
    SwerveAutoBuilder cmd = new SwerveAutoBuilder(pose::getRobotPose, pose::resetPose, SwerveConfig.SWERVE_KINEMATICS,
      new PIDConstants(7, 0.5, 0),
      new PIDConstants(9, 0.5, 0),
      swerve::setModuleStates, eventMap, true, swerve
    );

    finalCMD = cmd.fullAuto(traj);
    
    addRequirements(swerve);

  }
  @Override
  public void initialize() {

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