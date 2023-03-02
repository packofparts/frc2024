// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoMapConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.MoveByWithTarjectoryController;
import frc.robot.commands.PositionPIDtuning;
import frc.robot.commands.TGWithPPlib;
import frc.robot.commands.moveTo;
import frc.robot.subsystems.PoseEstimation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public SendableChooser <Command> autoSelector = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    PathPlannerTrajectory traj = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    m_robotContainer = new RobotContainer();
    autoSelector.addOption("Trajectory Classic", new AutonomousDrive(m_robotContainer.swerve));
    autoSelector.addOption("Auto Balance", new AutoBalanceCommand(m_robotContainer.swerve));
    autoSelector.addOption("Move By with Traj",
      new MoveByWithTarjectoryController(m_robotContainer.swerve, 
      new Transform2d(new Translation2d(2.5, 0), new Rotation2d(0))));
    autoSelector.addOption("Path Planner", new TGWithPPlib(m_robotContainer.swerve,AutoMapConstants.ConeCubeChargeTraj,AutoMapConstants.m_EventMap));
    autoSelector.addOption("ClassicMB", new moveTo(new Pose2d(0, 0, new Rotation2d(Math.PI/2)), m_robotContainer.swerve));
    autoSelector.addOption("AutoAlign", new AutoAlign(m_robotContainer.pose, m_robotContainer.lime, m_robotContainer.swerve));
    SmartDashboard.putData("Auto Path", autoSelector);
    autoSelector.addOption("PositionPID", new PositionPIDtuning(m_robotContainer.swerve, m_robotContainer.pose));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoSelector.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (m_robotContainer.joys.aprilAlign()) {
    //   m_robotContainer.align.schedule();
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
