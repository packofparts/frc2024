// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class SubstationAlignManual extends CommandBase {
  /** Creates a new SubstationAlignManual. */
  public final PIDController forwardController = new PIDController(0.2, 0, 0);
  public final PIDController rotationController = new PIDController(0.2, 0, 0);

  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  SwerveSubsystem swerve;
  LimelightPhoton lime;
  ArmControlSubsystem arm;
  ClawPnumatic claw;


  public SubstationAlignManual(SwerveSubsystem swerve, LimelightPhoton lime) {
    this.swerve = swerve;
    this.lime = lime;

    addRequirements(swerve);
    addRequirements(lime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhotonPipelineResult result = lime.getImg();
    double forwardSpeed;
    double rotationSpeed;

    if (result.hasTargets()) {
        // First calculate range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        VisionConstants.CAMERA_HEIGHT_METERS,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        VisionConstants.CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -rotationController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
        // If we have no targets, stay still.
        forwardSpeed = 0;
        rotationSpeed = 0;
    }
    swerve.setMotors(forwardSpeed, 0, rotationSpeed, DriveMode.AUTO, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardController.atSetpoint() && rotationController.atSetpoint();
  }
}
