// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightPhoton.Pipeline;


public class LimelightAlign extends CommandBase {
  public enum MovementMode{
    TRANSLATE_X,
    ROTATE,

  }
  /** Creates a new CubeAlign. */
  public SwerveSubsystem swerve;
  public LimelightPhoton lime;
  public double yaw;
  public Pipeline index;
  public double offset;
  public PIDController desiredPID;
  public PIDController secondaryPID;
  public MovementMode mode;
/**
 * 
 * @param swervesub
 * @param limelightsub
 * @param PipelineIndex Check pipeline enums in limelight sub
 * @param Xoffset THIS SHOULD BE IN METERS
 */
  public LimelightAlign(SwerveSubsystem swervesub, LimelightPhoton limelightsub, Pipeline pipeline,MovementMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = swervesub;
    this.mode = mode;
    lime = limelightsub;
    addRequirements(swerve);
    addRequirements(lime);
    index = pipeline;
    switch (mode){
      case TRANSLATE_X:
        desiredPID = PIDConstants.XController;
        desiredPID.setTolerance(PIDConstants.XControllerTolerance);
      case ROTATE:
        desiredPID = PIDConstants.rotController;
        desiredPID.setTolerance(PIDConstants.rotControllerTolerance);
    }
    
  }

    
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lime.setPipeline(index);


    if (lime.img.hasTargets()) {
      yaw = lime.getYaw();
    
      double outputSpeed = Units.degreesToRadians(desiredPID.calculate(yaw, 0));
      switch(this.mode){
        case TRANSLATE_X:
          swerve.setMotors(0,outputSpeed, 0);
        case ROTATE:
          swerve.setMotors(0, 0, outputSpeed);
      }

    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (desiredPID.atSetpoint()) {
      return true;
    }
    return false;
  }
}
