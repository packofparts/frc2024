package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveConfig;
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
  
  public FollowPath(SwerveSubsystem swerve, PathPlannerTrajectory traj, HashMap<String,Command> eventMap) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.eventMap = eventMap;
    this.traj = traj;
    
    addRequirements(this.swerve);
  }
  
  @Override
  public void initialize() {
    cmd = new SwerveAutoBuilder(this.swerve::getRobotPose, this.swerve::resetRobotPose, SwerveConfig.swerveKinematics,
            new PIDConstants(1, 0, 0), //old .4
            new PIDConstants(2, 0, 0), //old .5
            this.swerve::setModuleStates, this.eventMap, true, this.swerve
        );
    
    finalCMD = cmd.fullAuto(traj);

    SwerveSubsystem.autoGyroInitValue = traj.getInitialHolonomicPose().getRotation().getDegrees();

    finalCMD.schedule();
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