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
  private SwerveSubsystem _swerve;
  private Command _finalCMD;
  private PathPlannerTrajectory _traj;
  private Map<String,Command> _eventMap;
  private PoseEstimation _pose;
  
  public FollowPath(SwerveSubsystem swerve, PathPlannerTrajectory traj, Map<String,Command> eventMap, PoseEstimation pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this._swerve = swerve;
    this._eventMap = eventMap;
    this._traj = traj;
    this._pose = pose;
    
    addRequirements(this._swerve);

  }
  @Override
  public void initialize() {

    SwerveAutoBuilder cmd = new SwerveAutoBuilder(this._pose::getRobotPose, this._pose::resetPose, SwerveConfig.swerveKinematics,
            new PIDConstants(7, 0.5, 0), //old .4
            new PIDConstants(9, 0.5, 0), //old .5
            this._swerve::setModuleStates, this._eventMap, true, this._swerve
        );
    
    _finalCMD = cmd.fullAuto(_traj);

    SwerveSubsystem.autoGyroInitValue = _traj.getInitialHolonomicPose().getRotation().getDegrees();

    _finalCMD.schedule();
    SmartDashboard.putBoolean("pathFinished", _finalCMD.isFinished());
    SmartDashboard.putBoolean("pathSchedules", _finalCMD.isScheduled());
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
    return _finalCMD.isFinished();
  }
}