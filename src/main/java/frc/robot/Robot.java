// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths.AutoLeftMid;
import frc.robot.AutoPaths.AutoLeftHigh;
import frc.robot.AutoPaths.AutoRightHigh;
import frc.robot.AutoPaths.AutoRightMid;
import frc.robot.AutoPaths.MakeShiftAutoMiddle;
import frc.robot.AutoPaths.MobilityAuto;
import frc.robot.AutoPaths.MobilityCharge;
import frc.robot.AutoPaths.ScoreAndShit;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoMapConstants;
import frc.robot.Constants.AutoMapConstants.GamePiece;
import frc.robot.Constants.CompConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.MoveTo;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.PositionPIDtuning;
import frc.robot.commands.SubstationAlignMoveTo;
import frc.robot.commands.TGWithPPlib;
import frc.robot.commands.armcontrolcmds.ExtensionCmd;
import frc.robot.commands.armcontrolcmds.PivotCmd;
import frc.robot.commands.armcontrolcmds.ScoreConeHighNode;
import frc.robot.commands.armcontrolcmds.ScoreCubeHighNode;

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


    

    robotContainer = new RobotContainer();
    substationCmd = new SubstationAlignMoveTo(robotContainer.drivetrain, robotContainer.limeLightSubSystem);



    AutoMapConstants.populateHashMaps(robotContainer.drivetrain, robotContainer.limeLightSubSystem, robotContainer.arm, robotContainer.pose,robotContainer.claw);
    // PathPlannerTrajectory trajectory = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    //_commandSelector.addOption("Auto Balance", new AutoBalanceCommand(_robotContainer.drivetrain));
    // _commandSelector.addOption("Middle Auto",
    //     new MakeShiftAutoMiddle(_robotContainer.armControl, _robotContainer.clawPnumatic, _robotContainer.drivetrain));
    
    commandSelector.addOption("MiddleNodeAndCharge",
      new MakeShiftAutoMiddle(robotContainer.arm, robotContainer.claw, robotContainer.drivetrain));

    commandSelector.addOption("OnlyMobilitySide", 
      new MobilityAuto(robotContainer.drivetrain));
    
      commandSelector.addOption("MiddleNodeMobilityCharge",
      new MobilityCharge(robotContainer.drivetrain, robotContainer.arm, robotContainer.claw));

    commandSelector.addOption("ChargeStation",
      new AutoBalanceCommand(robotContainer.drivetrain));
    
    // commandSelector.addOption("MoveTo Pose Estimation", 
    //   new MoveTo(new Transform2d(new Translation2d(1, 0), new Rotation2d()), robotContainer.drivetrain, robotContainer.pose));


    // _commandSelector.addOption("SubstationAlignManuel", 
    //     new SubstationAlignManual(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem));
    // _commandSelector.addOption("SubstationAlignMoveTo", 
    //     new SubstationAlignMoveTo(_robotContainer.drivetrain, _robotContainer.limeLightSubSystem));  
    // _commandSelector.addOption("Mobility Charge <Self Destruct Sequence>", 
    //     new MobilityCharge(_robotContainer.drivetrain));
    commandSelector.addOption("Left High", 
        new AutoLeftHigh(robotContainer.arm, robotContainer.claw, robotContainer.drivetrain));   
    
    
    commandSelector.addOption("Right High", 
      new AutoRightHigh(robotContainer.arm, robotContainer.claw, robotContainer.drivetrain));


    commandSelector.addOption("Left Mid", 
      new AutoLeftMid(robotContainer.arm, robotContainer.claw, robotContainer.drivetrain));   
    
    
    commandSelector.addOption("Right Mid", 
      new AutoRightMid(robotContainer.arm, robotContainer.claw, robotContainer.drivetrain));
    
    commandSelector.addOption("MoveTo Pose", 
      new MoveTo(new Pose2d(3.565, 5.541, new Rotation2d(Units.degreesToRadians(177))), robotContainer.drivetrain, robotContainer.pose2));
    
    commandSelector.addOption("ScoreConeSit", new ScoreAndShit(robotContainer.arm, robotContainer.claw));


      commandSelector.addOption("PosPID", 
      new PositionPIDtuning(robotContainer.drivetrain));



    


      armModeSelector.addOption("Coast", ArmControlSubsystem.ArmMotorMode.COAST);
      armModeSelector.addOption("Brake", ArmControlSubsystem.ArmMotorMode.BRAKE);
      armModeSelector.addOption("Off", ArmControlSubsystem.ArmMotorMode.OFF);

     
      

      PPLIBPathSelector.addOption("Station2Piece", new SequentialCommandGroup(
        new ScoreConeHighNode(robotContainer.arm, robotContainer.claw),
        new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.station2Piece, AutoMapConstants.eventMap),
        new ScoreCubeHighNode(robotContainer.arm, robotContainer.claw),
        new InstantCommand(() -> robotContainer.claw.closePneumatics(), robotContainer.claw),
        new ExtensionCmd(robotContainer.arm, 0),
        new PivotCmd(robotContainer.arm, ArmConstants.minAngleRad)

      ));

      PPLIBPathSelector.addOption("Bump2Piece", new SequentialCommandGroup(
        new InstantCommand(() -> SwerveSubsystem.resetGyro()),
        new PivotCmd(robotContainer.arm, ArmConstants.angleLevelsRad[2]),
        new ExtensionCmd(robotContainer.arm, ArmConstants.extensionLevelsIn[2]),
        new WaitCommand(.3),
        robotContainer.claw.dropPiece(GamePiece.CONE),
        new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.bump2Piece, AutoMapConstants.eventMap),
        new ScoreCubeHighNode(robotContainer.arm, robotContainer.claw),
        new InstantCommand(() -> robotContainer.claw.closePneumatics(), robotContainer.claw),
        new ExtensionCmd(robotContainer.arm, 0),
        new PivotCmd(robotContainer.arm, ArmConstants.minAngleRad)

      ));

      PPLIBPathSelector.addOption("Station2PieceCharge", new SequentialCommandGroup(
        new ScoreConeHighNode(robotContainer.arm, robotContainer.claw),
        new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.station2Piece, AutoMapConstants.eventMap),
        new ScoreCubeHighNode(robotContainer.arm, robotContainer.claw),
        new InstantCommand(() -> robotContainer.claw.closePneumatics(), robotContainer.claw),
        new ExtensionCmd(robotContainer.arm, 0),
        new PivotCmd(robotContainer.arm, ArmConstants.minAngleRad),
        new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.station2Piece, AutoMapConstants.eventMap)
      ));

      PPLIBPathSelector.addOption("ConeCubeBump", new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.ConeCubeBump, AutoMapConstants.emptyMap));
      PPLIBPathSelector.addOption("ConeCubeBarrier", new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.ConeCubeBarrier, AutoMapConstants.emptyMap));
      PPLIBPathSelector.addOption("ConeCubeChargeBump",  new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.ConeCubeChargeBump, AutoMapConstants.emptyMap));

      //PPLIBPathSelector.addOption("OneMeterForward", new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.move1Meter, AutoMapConstants.emptyMap));
      //PPLIBPathSelector.addOption("TwoMeter+180",  new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.move1MeterRotate, AutoMapConstants.emptyMap));
      //PPLIBPathSelector.addOption("backForth",  new TGWithPPlib(robotContainer.drivetrain, AutoMapConstants.backforth, AutoMapConstants.emptyMap));


    //commandSelector.addOption("PositionPID", new PositionPIDtuning(robotContainer.drivetrain));

    SmartDashboard.putData("PPLib Paths", PPLIBPathSelector);
    SmartDashboard.putData("ResetArmEncoders", new InstantCommand(() -> robotContainer.arm.resetEncoders()));
    SmartDashboard.putData("ArmModes", armModeSelector);
    SmartDashboard.putData("Auto commands", commandSelector);
    
    SmartDashboard.putData("Coast Drive", new InstantCommand(()->robotContainer.drivetrain.setIdleModeForAll(IdleMode.kCoast,IdleMode.kCoast)));
    SmartDashboard.putData("Brake Drive", new InstantCommand(()->robotContainer.drivetrain.setIdleModeForAll(IdleMode.kBrake,IdleMode.kBrake)));

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
    if (CompConstants.debug && DriverStation.isFMSAttached()) {
      CompConstants.debug = false;
    }
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
    // if (PPLIBPathSelector.getSelected() != null){
       //autonomousCommand = PPLIBPathSelector.getSelected();
    // } else{
      autonomousCommand = commandSelector.getSelected();
    // }

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
    // if(Input.limelightAlignTrigger()){

    //   if(isOn){
    //     substationCmd.cancel();
        
    //     isOn = false;
    //   }else{
    //     isOn = true;
    //     substationCmd = new  SubstationAlignMoveTo(robotContainer.drivetrain, robotContainer.limeLightSubSystem);
    //     substationCmd.schedule();
    //   }

    // }


    // SmartDashboard.putBoolean("SubStation", substationCmd.isScheduled());

    // SmartDashboard.putBoolean("is On", isOn);
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
