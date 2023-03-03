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
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.PIDConstants;


public class PositionPIDtuning extends CommandBase {
  /** Creates a new PositionPIDtuning. */
  SwerveSubsystem swerve;
  Translation2d desiredTrans;
  PoseEstimation pEstimation;
  moveTo moveTo;
  public PositionPIDtuning(SwerveSubsystem swerve, PoseEstimation pose) {
    
    this.swerve = swerve;
    this.pEstimation = pose;
    // Use addRequirements() here to declare subsystem dependencies.
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Run Translation?", false);
    SmartDashboard.putNumber("X translation meters", 0);
    SmartDashboard.putNumber("Y translation meters", 0);
    SmartDashboard.putNumber("Change in Rotation degrees", 0);
    // SmartDashboard.putNumberArray("Translation PID controller",
    // new double [] {
    //   PIDConstants.XController.getP(),
    //   PIDConstants.XController.getI(),
    //   PIDConstants.XController.getD()});

    SmartDashboard.putNumber("XControllerP", PIDConstants.XController.getP());
    SmartDashboard.putNumber("XControllerI", PIDConstants.XController.getI());
    SmartDashboard.putNumber("XControllerD", PIDConstants.XController.getD());

    // SmartDashboard.putNumberArray("Rotation PID controller",
    // new double [] {
    //   PIDConstants.XController.getP(),
    //   PIDConstants.XController.getI(),
    //   PIDConstants.XController.getD()});

    SmartDashboard.putNumber("RotControllerP", PIDConstants.rotController.getP());
    SmartDashboard.putNumber("RotControllerI", PIDConstants.rotController.getI());
    SmartDashboard.putNumber("RotControllerD", PIDConstants.rotController.getD());


    moveTo = new moveTo(
      new Transform2d(
        new Translation2d(SmartDashboard.getNumber("X translation meters", 0),
        SmartDashboard.getNumber("Y translation meters", 0)),
        new Rotation2d(SmartDashboard.getNumber("Change in Rotation degrees", 0))
      ), swerve,pEstimation);
      SmartDashboard.putBoolean("moveToIsScheduled?", moveTo.isScheduled());
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("moveToIsScheduled?", moveTo.isScheduled());
    PIDConstants.XController = new PIDController(
      SmartDashboard.getNumber("XControllerP", 0),
      SmartDashboard.getNumber("XControllerI", 0),
      SmartDashboard.getNumber("XControllerD", 0)
      );
    PIDConstants.YController = PIDConstants.XController;
    PIDConstants.rotController = new PIDController(
      SmartDashboard.getNumber("RotControllerP", 0),
      SmartDashboard.getNumber("RotControllerI", 0),
      SmartDashboard.getNumber("RotControllerD", 0)
      );
    if (moveTo != null && true && moveTo.isFinished() && !moveTo.isScheduled()){
      moveTo = new moveTo(
        new Transform2d(
          new Translation2d(SmartDashboard.getNumber("X translation meters", 0),
          SmartDashboard.getNumber("Y translation meters", 0)),
          new Rotation2d(SmartDashboard.getNumber("Change in Rotation degrees", 0))
        ), swerve, pEstimation);
        SmartDashboard.putBoolean("Run Translation", false);
        moveTo.schedule();
        SmartDashboard.putBoolean("moveToIsSchduled", moveTo.isScheduled());
        SmartDashboard.putBoolean("Run Translation?", false);
    }

    
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
