/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.DefaultDriveCmd;
import frc.robot.commands.PIDtuning;
import frc.robot.commands.SinglePID;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Joysticks joys = new Joysticks();
  public final SwerveSubsystem swerve = new SwerveSubsystem(joys);
  public final DefaultDriveCmd npc = new DefaultDriveCmd(joys, swerve);
  public final PIDtuning pud = new PIDtuning(joys,swerve);

  public SendableChooser <SwerveModule> moduleSelector = new SendableChooser<>();
  public SwerveModule [] allModules = swerve.getRawModules(); 
  public SwerveModule selecModule = allModules[1];


  

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

    if (!Constants.tuningPID){swerve.setDefaultCommand(npc);}
    else{swerve.setDefaultCommand(new SinglePID(selecModule, swerve));}
  
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
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonomousDrive(swerve);//m_autoCommand;
  }
}
