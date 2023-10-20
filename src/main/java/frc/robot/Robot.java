// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPath;
import frc.robot.commands.Mobility;
import frc.robot.commands.ScoreCone;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ArmConstants.ArmState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command _autonomousCommand;

  private RobotContainer _robotContainer;

  public SendableChooser<Command> pathSelector = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    _robotContainer = new RobotContainer();


    pathSelector.addOption("NONE", null);
    pathSelector.addOption("Station2PieceBlue", 
        new SequentialCommandGroup(
        new ScoreCone(_robotContainer.armControlSubsystem, _robotContainer.intakeSubsystem, ArmState.LOWER_NODE_CONE),
        new FollowPath(
          _robotContainer.swerveSubsystem, 
          AutoConstants.station2PieceBlue,
          AutoConstants.eventMap,
          _robotContainer.pose
        )
        
      )
    );

    pathSelector.addOption("Station2PieceRed", 
    new SequentialCommandGroup(
    new ScoreCone(_robotContainer.armControlSubsystem, _robotContainer.intakeSubsystem, ArmState.LOWER_NODE_CONE),
    new FollowPath(
      _robotContainer.swerveSubsystem, 
      AutoConstants.station2PieceRed,
      AutoConstants.eventMap,
      _robotContainer.pose
    )
    
    )
  );
    
    pathSelector.addOption("ScoreHybridMobility", 
      new SequentialCommandGroup(
        new ScoreCone(_robotContainer.armControlSubsystem, _robotContainer.intakeSubsystem, ArmState.LOWER_NODE_CONE),
        new Mobility(_robotContainer.swerveSubsystem)
      )
    );

    pathSelector.addOption("ScoreHybrid", 
      new SequentialCommandGroup(
        new ScoreCone(_robotContainer.armControlSubsystem, _robotContainer.intakeSubsystem, ArmState.LOWER_NODE_CONE)
      )
    );

    pathSelector.addOption("OneMeter", new FollowPath(
      _robotContainer.swerveSubsystem, 
      AutoConstants.moveOneMeters,
      AutoConstants.emptyMap,
      _robotContainer.pose
    ));


    SmartDashboard.putData("PP Autos", pathSelector);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    _autonomousCommand = _robotContainer.getAutonomousCommand();

    if(pathSelector.getSelected() != null){
      _autonomousCommand = pathSelector.getSelected();
    }

    // schedule the autonomous command (example)
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();
    }
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //empty for now but will add auton paths here
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //We will add things to here shortly
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
  }
}
