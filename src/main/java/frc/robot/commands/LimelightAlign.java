// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
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
    rotPID = new PIDController(3, 0, 0);
    rotPID.setTolerance(0.001);
    offset = Xoffset;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lime.setPipeline(index);
    double ballDist = lime.getForwardDistance(0.7);
    double cubeSize = lime.getSize();
    double sp = -(0.983*cubeSize-7.2608);

    if (lime.img.hasTargets()) {
      yaw = lime.getYaw();

      double rotSpeed = Units.degreesToRadians(rotPID.calculate(yaw, sp));
      swerve.setMotors(0,0, rotSpeed);
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
