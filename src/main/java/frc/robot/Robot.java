// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoPaths.AutoLeftMid;
import frc.robot.AutoPaths.AutoLeftHigh;
import frc.robot.AutoPaths.AutoRightHigh;
import frc.robot.AutoPaths.AutoRightMid;
import frc.robot.AutoPaths.MakeShiftAutoMiddle;
import frc.robot.AutoPaths.MobilityAuto;
import frc.robot.AutoPaths.MobilityCharge;
import frc.robot.Constants.AutoMapConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.MoveTo;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.commands.PositionPIDtuning;
import frc.robot.commands.SubstationAlignManual;
import frc.robot.commands.SubstationAlignMoveTo;
import frc.robot.commands.TGWithPPlib;

public class Robot extends TimedRobot {
  
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public SendableChooser <Command> commandSelector = new SendableChooser<>();
  public SendableChooser <Command> PPLIBPathSelector = new SendableChooser<>();
  public static SendableChooser <ArmControlSubsystem.ArmMotorMode> armModeSelector = new SendableChooser<>();


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


    robotContainer = new RobotContainer();
    substationCmd = new SubstationAlignMoveTo(robotContainer.drivetrain, robotContainer.limeLightSubSystem);


    // PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    //_commandSelector.addOption("Auto Balance", new AutoBalanceCommand(_robotContainer.drivetrain));
    // _commandSelector.addOption("Middle Auto",
    //     new MakeShiftAutoMiddle(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));
    
    commandSelector.addOption("Middle",
      new MakeShiftAutoMiddle(robotContainer.armControl, robotContainer.clawPnumatic, robotContainer.drivetrain));

    commandSelector.addOption("OnlyMobilitySide", 
      new MobilityAuto(robotContainer.drivetrain));
    


    commandSelector.addOption("ChargeStation",
      new AutoBalanceCommand(robotContainer.drivetrain));
    
    commandSelector.addOption("MoveTo Pose Estimation", 
      new MoveTo(new Transform2d(new Translation2d(1, 0), new Rotation2d()), robotContainer.drivetrain, robotContainer.pose));


    // _commandSelector.addOption("SubstationAlignManuel", 
    //     new SubstationAlignManual(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem));
    // _commandSelector.addOption("SubstationAlignMoveTo", 
    //     new SubstationAlignMoveTo(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem));  
    // _commandSelector.addOption("Mobility Charge <Self Destruct Sequence>", 
    //     new MobilityCharge(_robotContainer.drivetrain));
    commandSelector.addOption("Left High", 
        new AutoLeftHigh(robotContainer.armControl, robotContainer.clawPnumatic, robotContainer.drivetrain));   
    
    
    commandSelector.addOption("Right High", 
      new AutoRightHigh(robotContainer.armControl, robotContainer.clawPnumatic, robotContainer.drivetrain));


    commandSelector.addOption("Left Mid", 
      new AutoLeftMid(robotContainer.armControl, robotContainer.clawPnumatic, robotContainer.drivetrain));   
    
    
    commandSelector.addOption("Right Mid", 
      new AutoRightMid(robotContainer.armControl, robotContainer.clawPnumatic, robotContainer.drivetrain));
    
        // _commandSelector.addOption(
    //   "Move By with Trajecotry",
    //   new MoveByWithTrajectoryController(
    //     _robotContainer.drivetrain, 
    //     new Transform2d(
    //       new Translation2d(2.5, 0), 
    //       new Rotation2d(0))));
    
    commandSelector.addOption(
      "Path Planner FULL PATH", 
      new TGWithPPlib(
        robotContainer.drivetrain,
        AutoMapConstants.ConeCubeChargeTraj,
        AutoMapConstants.m_EventMap));

    commandSelector.addOption(
      "Path Planner Only DRIVE", 
      new TGWithPPlib(
      robotContainer.drivetrain,
        AutoMapConstants.ConeCubeChargeTraj,
        AutoMapConstants.m_EventMap));

    commandSelector.addOption(
      "Path Planner Only DRIVE", 
      new TGWithPPlib(
    robotContainer.drivetrain,
        AutoMapConstants.ConeCubeChargeTraj,
        AutoMapConstants.m_EventMap));    
    
    
    // _commandSelector.addOption(
    //   "Test Motor",
    //   new TestSpark(7, -0.3));
    


      armModeSelector.addOption("Coast", ArmControlSubsystem.ArmMotorMode.COAST);
      armModeSelector.addOption("Brake", ArmControlSubsystem.ArmMotorMode.BRAKE);
      armModeSelector.addOption("Off", ArmControlSubsystem.ArmMotorMode.OFF);

      PPLIBPathSelector.addOption("ConeNodeToCube", autonomousCommand);


    //commandSelector.addOption("PositionPID", new PositionPIDtuning(robotContainer.drivetrain));


    SmartDashboard.putData("ResetArmEncoders", new InstantCommand(() -> robotContainer.armControl.resetEncoders()));
    SmartDashboard.putData("ArmModes", armModeSelector);
    SmartDashboard.putData("Auto commands", commandSelector);
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
    autonomousCommand = commandSelector.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    

    //CANSparkMaxLowLevel.enableExternalUSBControl(false);
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
        substationCmd = new  SubstationAlignMoveTo(robotContainer.drivetrain, robotContainer.limeLightSubSystem);
        substationCmd.schedule();
      }

    }


    SmartDashboard.putBoolean("SubStation", substationCmd.isScheduled());

    SmartDashboard.putBoolean("is On", isOn);
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
