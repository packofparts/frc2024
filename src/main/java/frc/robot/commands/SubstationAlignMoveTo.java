// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmControlSubsystem.ArmSetting;

public class SubstationAlignMoveTo extends CommandBase {


  private final Pose2d relativeDesiredPose = new Pose2d(new Translation2d(-4, -0.5), new Rotation2d(0));
  private final Transform3d offset = new Transform3d(new Translation3d(-2, 0, 0), new Rotation3d(0, 0, 0));


  private SwerveSubsystem swerve;
  private Limelight lime;

  ArmControlSubsystem arm;
  ClawPnumatic claw;
  boolean failed = false;
  
  private SequentialCommandGroup commandGroup;
  private PoseEstimation pose;

  private MoveTo move;

  final double GOAL_RANGE_METERS = Units.feetToMeters(3);


  /** Creates a new SubstationAlign. */
  public SubstationAlignMoveTo(SwerveSubsystem swerve, Limelight lime) {
    this.swerve = swerve;
    this.lime = lime;


    addRequirements(swerve);
    addRequirements(lime);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhotonPipelineResult result = lime.getImg();

    if (result.hasTargets()) {

      Transform3d transformation = result.getBestTarget().getBestCameraToTarget();

      Transform2d fin = new Transform2d(new Translation2d(0, transformation.getY()), new Rotation2d(0));//-swerve.getRotation2d().getRadians()));

      move = new MoveTo(fin, swerve);

      SmartDashboard.putNumber("TransformX",fin.getX());
      SmartDashboard.putNumber("TransformY", fin.getY());
      SmartDashboard.putNumber("TransformRot", fin.getRotation().getDegrees());
      
      commandGroup = new SequentialCommandGroup(
        move
      );

      if (commandGroup != null) {
        commandGroup.schedule();
        
      } else {
        failed = true;
        System.out.println("bababoey");
      }


    
    
    }
  }

  public double optimize(double rad) {
    //return rad%Math.PI - Math.PI * (rad/Math.abs(rad));
    return rad;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //failed || commandGroup.isFinished();
  }
}
