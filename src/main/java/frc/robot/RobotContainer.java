/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.commands.AimbotDriveCmd;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCmd;
import frc.robot.commands.PIDtuning;
import frc.robot.commands.SinglePID;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //subsystems
  public final SwerveSubsystem swerve = new SwerveSubsystem();
  
  public final Limelight lime = new Limelight();
  public final PoseEstimation pose = new PoseEstimation(lime, swerve);

  //commented because testing and probably will cause null errors
  //public final ArmControlSubsystem armControl = new ArmControlSubsystem();
  //public final ClawMotor clawMotor = new ClawMotor();

  //commands
  public final DefaultDriveCmd defaultDrive = new DefaultDriveCmd(swerve);
  public final AimbotDriveCmd aimbot = new AimbotDriveCmd(swerve, lime);
  public final PIDtuning pid = new PIDtuning(swerve);
  public final AutoAlign align = new AutoAlign(pose, lime, swerve, new Transform2d(new Translation2d(1, 0), new Rotation2d(0)));


  public SendableChooser <SwerveModule> moduleSelector = new SendableChooser<>();

  public SwerveModule [] allModules = swerve.getRawModules(); 
  public SwerveModule selecModule = allModules[3];


  

  //public final pnumatics pnu = new pnumatics();
  //public final Camera cam = new Camera();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    moduleSelector.addOption("Front Left", allModules[0]);
    moduleSelector.addOption("Front Right", allModules[1]);
    moduleSelector.addOption("Back Left", allModules[2]);
    moduleSelector.addOption("Back Right", allModules[3]);


    if (!DriveConstants.tuningPID){swerve.setDefaultCommand(defaultDrive);}
    else{swerve.setDefaultCommand(new SinglePID(selecModule, swerve));}
  
    //armControl.setDefaultCommand(new DefaultArmCommand(armControl));

    SmartDashboard.putData("CHOOOSE", moduleSelector);
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(Input.getTJoystick(), 2).toggleOnTrue(aimbot);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;//m_autoCommand;
  }
}
