// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.PoseEstimation;
import frc.robot.Constants.PIDConstants;


public class PositionPIDtuning extends CommandBase {
  /** Creates a new PositionPIDtuning. */
  SwerveSubsystem swerve;
  Translation2d desiredTrans;
  PoseEstimation pEstimation;
  MoveTo back, forth;
  SequentialCommandGroup backAndForth;

  public PositionPIDtuning(SwerveSubsystem swerve) {
    
    this.swerve = swerve;
    //this.pEstimation = pose;
    // Use addRequirements() here to declare subsystem dependencies.
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Run Translation?", false);
    SmartDashboard.putNumber("X Translation", 0);
    SmartDashboard.putNumber("Y Translation", 0);
    SmartDashboard.putNumber("Change in Rotation degrees", 0);


    SmartDashboard.putNumber("TransControllerP", PIDConstants.transPIDValues[0]);
    SmartDashboard.putNumber("TransControllerI", PIDConstants.transPIDValues[1]);
    SmartDashboard.putNumber("TransControllerD", PIDConstants.transPIDValues[2]);


    SmartDashboard.putNumber("RotControllerP", PIDConstants.rotPIDValues[0]);
    SmartDashboard.putNumber("RotControllerI", PIDConstants.rotPIDValues[1]);
    SmartDashboard.putNumber("RotControllerD", PIDConstants.rotPIDValues[2]);


    PIDConstants.transPIDValues[0] = SmartDashboard.getNumber("TransControllerP", 0);
    PIDConstants.transPIDValues[1] = SmartDashboard.getNumber("TransControllerI", 0);
    PIDConstants.transPIDValues[2] = SmartDashboard.getNumber("TransControllerD", 0);

    PIDConstants.rotPIDValues[0] = SmartDashboard.getNumber("RotControllerP", 0);
    PIDConstants.rotPIDValues[1] = SmartDashboard.getNumber("RotControllerI", 0);
    PIDConstants.rotPIDValues[2] = SmartDashboard.getNumber("RotControllerD", 0);


    forth = new MoveTo(new Transform2d(new Translation2d(2, 0), new Rotation2d(0)), swerve);
    back = new MoveTo(new Transform2d(new Translation2d(-2, 0), new Rotation2d(0)), swerve);
    backAndForth = new SequentialCommandGroup(forth, back);
    
    
    backAndForth.repeatedly().schedule();

    //forth.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    //try moving to a subsystem if it doesnt work
    //SmartDashboard.putBoolean("BackAndForchIsScheduled", backAndForth.isScheduled());
    SmartDashboard.putBoolean("ForthIsScheduled", forth.isScheduled());
    SmartDashboard.putBoolean("BackIsScheduled", back.isScheduled());



    // PIDConstants.XController = new PIDController(
    //   SmartDashboard.getNumber("XControllerP", 0),
    //   SmartDashboard.getNumber("XControllerI", 0),
    //   SmartDashboard.getNumber("XControllerD", 0)
    //   );



    // PIDConstants.YController = PIDConstants.XController;
    // PIDConstants.rotController = new PIDController(
    //   SmartDashboard.getNumber("RotControllerP", 0),
    //   SmartDashboard.getNumber("RotControllerI", 0),
    //   SmartDashboard.getNumber("RotControllerD", 0)
    //   );



    // if (moveTo != null && true && moveTo.isFinished() && !moveTo.isScheduled()){
    //   moveTo = new moveTo(
    //     new Transform2d(
    //       new Translation2d(SmartDashboard.getNumber("X translation meters", 0),
    //       SmartDashboard.getNumber("Y translation meters", 0)),
    //       new Rotation2d(SmartDashboard.getNumber("Change in Rotation degrees", 0))
    //     ), swerve, pEstimation);
    //     SmartDashboard.putBoolean("Run Translation", false);
    //     moveTo.schedule();
    //     SmartDashboard.putBoolean("moveToIsSchduled", moveTo.isScheduled());
    //     SmartDashboard.putBoolean("Run Translation?", false);
    // }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
