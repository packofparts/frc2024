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
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCmd;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.PoseEstimation;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  
  public final Limelight limeLightSubSystem = new Limelight();

  public final PoseEstimation pose = new PoseEstimation(limeLightSubSystem, drivetrain);
  //public final ManualPoseEstimation pose = new ManualPoseEstimation(limeLightSubSystem, drivetrain, ManualPoseEstimation.Strategy.BEST);

  
  public final ArmControlSubsystem armControl = new ArmControlSubsystem();
  public final ClawPnumatic clawPnumatic = new ClawPnumatic();

  // Commands
  public final DefaultDriveCmd defaultDrive = new DefaultDriveCmd(drivetrain);
  //public final AimbotDriveCmd aimbot = new AimbotDriveCmd(drivetrain, limeLightSubSystem);
  //public final PIDtuning pid = new PIDtuning(drivetrain);

  //public final AutoAlign align = 
   // new AutoAlign(pose, lime, swerve, new Transform2d(new Translation2d(1, 0), new Rotation2d(0)));

  // public final LimelightAlign generalAlign = 
  //   new LimelightAlign(
  //     drivetrain, 
  //     limeLightSubSystem, 
  //     1, 
  //     0);

  public SendableChooser <SwerveModule> moduleSelector = new SendableChooser<>();

  //public SwerveModule [] allModules = drivetrain.getRawModules(); 
  //public SwerveModule selecModule = allModules[2];


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // moduleSelector.addOption("Front Left", allModules[0]);
    // moduleSelector.addOption("Front Right", allModules[1]);
    // moduleSelector.addOption("Back Left", allModules[2]);
    // moduleSelector.addOption("Back Right", allModules[3]);
    drivetrain.updateAbsEncOffsets();
    if (!DriveConstants.tuningPID){
      drivetrain.setDefaultCommand(defaultDrive);
    } else{
      // drivetrain.setDefaultCommand(
      //   new SinglePID(
      //     selecModule 
      //     ));
    }
  
    armControl.setDefaultCommand(new DefaultArmCommand(armControl));

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
    //new JoystickButton(Input.getTJoystick(), 2).toggleOnTrue(aimbot);
  }
}
