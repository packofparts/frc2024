package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.SwerveConfig;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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

    // SwerveAutoBuilder cmd = new SwerveAutoBuilder(mPoseEstimator::getRobotPose,
    // mPoseEstimator::resetPose, SwerveConfig.SWERVE_KINEMATICS, new PIDConstants(7, 0.5, 0),
    // new PIDConstants(9, 0.5, 0), mSwerve::setModuleStates, mEventMap, true, mSwerve);

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(mPoseEstimator::getRobotPose, mPoseEstimator::resetPose,
        swerve::getChassisSpeeds,
        ChassisSpeeds -> swerve.setChassisSpeed(swerve.getChassisSpeeds()),
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
                                         // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            3, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
                                   // here
        ), swerve);

    PathPlannerPath path = PathPlannerPath.fromPathFile("MoveOneMeter");
    mFinalCMD = AutoBuilder.followPathWithEvents(path);
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
