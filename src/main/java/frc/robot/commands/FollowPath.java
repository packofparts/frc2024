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
  private final SwerveSubsystem mSwerve;
  private final Command mFinalCMD;
  private final PathPlannerTrajectory mTraj;
  private final Map<String, Command> mEventMap;
  private final PoseEstimation mPoseEstimator;

  public FollowPath(SwerveSubsystem swerve, PathPlannerTrajectory traj,
      Map<String, Command> eventMap, PoseEstimation poseEstimator) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = swerve;
    mEventMap = eventMap;
    mTraj = traj;
    mPoseEstimator = poseEstimator;

    SwerveAutoBuilder cmd = new SwerveAutoBuilder(mPoseEstimator::getRobotPose,
        mPoseEstimator::resetPose, SwerveConfig.SWERVE_KINEMATICS, new PIDConstants(7, 0.5, 0),
        new PIDConstants(9, 0.5, 0), mSwerve::setModuleStates, mEventMap, true, mSwerve);

    mFinalCMD = cmd.fullAuto(mTraj);

    addRequirements(mSwerve);

  }

  @Override
  public void initialize() {
    mFinalCMD.schedule();
    SmartDashboard.putBoolean("pathFinished", mFinalCMD.isFinished());
    SmartDashboard.putBoolean("pathSchedules", mFinalCMD.isScheduled());
  }

  @Override
  public void execute() {
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
  public boolean isFinished() {
    return mFinalCMD.isFinished();
  }
}
