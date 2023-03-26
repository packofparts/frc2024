// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoPaths.AutoLeftMid;
import frc.robot.AutoPaths.AutoLeftHigh;
import frc.robot.AutoPaths.AutoRightHigh;
import frc.robot.AutoPaths.AutoRightMid;
import frc.robot.AutoPaths.MakeShiftAutoMiddle;
import frc.robot.AutoPaths.MobilityAuto;
import frc.robot.AutoPaths.MobilityCharge;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.Input;
import frc.robot.commands.PositionPIDtuning;
import frc.robot.commands.SubstationAlignManual;
import frc.robot.commands.SubstationAlignMoveTo;

public class Robot extends TimedRobot {
  
  private Command _autonomousCommand;
  private RobotContainer _robotContainer;
  public SendableChooser <Command> _commandSelector = new SendableChooser<>();
  public SubstationAlignMoveTo substationCmd;

  boolean isOn = false;

  //Compressor phCompressor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    CameraServer.startAutomaticCapture();


    _robotContainer = new RobotContainer();
    substationCmd = new SubstationAlignMoveTo(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem);


    // PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    //_commandSelector.addOption("Auto Balance", new AutoBalanceCommand(_robotContainer.drivetrain));
    // _commandSelector.addOption("Middle Auto",
    //     new MakeShiftAutoMiddle(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));
    
    _commandSelector.addOption("Middle",
        new MakeShiftAutoMiddle(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));

    _commandSelector.addOption("OnlyMobilitySide", 
        new MobilityAuto(_robotContainer.drivetrain));
    


    _commandSelector.addOption("ChargeStation",
         new AutoBalanceCommand(_robotContainer.drivetrain));


    // _commandSelector.addOption("SubstationAlignManuel", 
    //     new SubstationAlignManual(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem));
    // _commandSelector.addOption("SubstationAlignMoveTo", 
    //     new SubstationAlignMoveTo(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem));  
    // _commandSelector.addOption("Mobility Charge <Self Destruct Sequence>", 
    //     new MobilityCharge(_robotContainer.drivetrain));
    _commandSelector.addOption("Left High", 
        new AutoLeftHigh(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));   
    
    
        _commandSelector.addOption("Right High", 
        new AutoRightHigh(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));


        _commandSelector.addOption("Left Mid", 
        new AutoLeftMid(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));   
    
    
        _commandSelector.addOption("Right Mid", 
        new AutoRightMid(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));
    
        // _commandSelector.addOption(
    //   "Move By with Trajecotry",
    //   new MoveByWithTrajectoryController(
    //     _robotContainer.drivetrain, 
    //     new Transform2d(
    //       new Translation2d(2.5, 0), 
    //       new Rotation2d(0))));
    
    // _commandSelector.addOption(
    //   "Path Planner", 
    //   new TGWithPPlib(
    //     _robotContainer.drivetrain,
    //     AutoMapConstants.ConeCubeChargeTraj,
    //     AutoMapConstants.m_EventMap));
    // _commandSelector.addOption(
    //   "Test Motor",
    //   new TestSpark(7, -0.3));
    
    

    
    // _commandSelector.addOption(
    //   "ClassicMB", 
    //   new MoveTo(
    //     new Pose2d(0, 0, 
    //     new Rotation2d(Math.PI/2)), 
    //   _robotContainer.drivetrain, 
    //   _robotContainer.poseEstimator));
    
    // autoSelector.addOption("AutoAlign", new AutoAlign(m_robotContainer.pose, m_robotContainer.lime, m_robotContainer.swerve));
    _commandSelector.addOption("PositionPID", new PositionPIDtuning(_robotContainer.drivetrain));

    // _commandSelector.addOption(
    //   "LimelightAlign", 
    //   new LimelightAlign(
    //     _robotContainer.drivetrain, 
    //     _robotContainer.limeLightSubSystem, 
    //     1, 
    //     0));

    SmartDashboard.putData("Auto commands", _commandSelector);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test. fr
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
    

    // phCompressor.enableDigital();
    // if (phCompressor.getPressure() > 60) phCompressor.disable();
    // else if (phCompressor.getPressure() < 60) phCompressor.enableDigital();
  }

  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    _autonomousCommand = _commandSelector.getSelected();

    // schedule the autonomous command (example)
    if (_autonomousCommand != null) {
      _autonomousCommand.schedule();
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
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();
    }

    

    CANSparkMaxLowLevel.enableExternalUSBControl(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // if(Input.LimelightAlignTrigger()){
    //   susStation = new SubstationAlignMoveTo(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem);
    //   susStation.schedule();
    // }else if (Input.LimelightAlignTrigger() && susStation.isScheduled()){
    //   susStation.cancel();
    // }

    if(Input.limelightAlignTrigger()){

      if(isOn){
        substationCmd.cancel();
        
        isOn = false;
      }else{
        isOn = true;
        substationCmd = new  SubstationAlignMoveTo(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem);
        substationCmd.schedule();
      }

    }


    SmartDashboard.putBoolean("SusStation", substationCmd.isScheduled());

    SmartDashboard.putBoolean("ISON", isOn);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
