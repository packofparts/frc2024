// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight.Pipeline;


public class LimelightAlign extends CommandBase {
  /** Creates a new CubeAlign. */
  public SwerveSubsystem swerve;
  public Limelight lime;
  public double yaw;
  public int index;
  public double offset;
  public PIDController rotPID;
/**
 * 
 * @param swervesub
 * @param limelightsub
 * @param PipelineIndex Check pipeline enums in limelight sub
 * @param Xoffset THIS SHOULD BE IN METERS
 */
  public LimelightAlign(SwerveSubsystem swervesub, Limelight limelightsub, int PipelineIndex, double Xoffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = swervesub;
    lime = limelightsub;
    addRequirements(swerve);
    addRequirements(lime);
    index = PipelineIndex;
    rotPID = new PIDController(0.5, 0, 0);
    rotPID.setTolerance(0.1);
    offset = Xoffset;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lime.setPipeline(index);
    if (lime.img.hasTargets()) {
      yaw = lime.getXoffset();
      swerve.setMotors(0,rotPID.calculate(yaw,offset),0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rotPID.atSetpoint()) {
      return true;
    }
    return false;
  }
}
