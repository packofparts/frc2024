// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawMotor;



public class AutonomousCommand extends CommandBase {
  // UPDATE WITH POSES OF PIECES
  public Pose2d redLeftPiece = new Pose2d();
  public Pose2d redRightPiece = new Pose2d();
  public Pose2d blueLeftPiece = new Pose2d();
  public Pose2d blueRightPiece = new Pose2d();


  public static enum paths {
    REDLEFT,
    REDRIGHT,
    REDMIDDLE,
    BLUELEFT,
    BLUERIGHT,
    BLUEMIDDLE
  }

  /** Creates a new AutonomousCommand. */
  public Limelight lime;
  public SwerveSubsystem swerve;
  public moveTo move;
  public double desiredOffset = 12;
  public boolean gotOffset;
  public boolean movedForward;
  public boolean turned;
  public PoseEstimation poseEstimator;
  public Pose2d firstPiece;
  public ArmControlSubsystem arm;
  public ClawMotor claw;
  public SequentialCommandGroup command;


  public AutonomousCommand(Limelight limeSub, SwerveSubsystem swerveSub, ArmControlSubsystem armSub, ClawMotor clawSub, PoseEstimation poseEstimatorr, paths path) {
    lime = limeSub;
    swerve = swerveSub;
    poseEstimator = poseEstimatorr;
    arm = armSub;
    claw = clawSub;
    addRequirements(swerve);
    addRequirements(lime);
    addRequirements(arm);
    addRequirements(claw);
    gotOffset = false;
    movedForward = false;

    boolean middle = false;
    switch(path) {
      case REDLEFT:
        firstPiece = redLeftPiece;
        break;
      case REDRIGHT:
        firstPiece = redRightPiece;
        break;
      case REDMIDDLE:
        middle = true;
        break;
      case BLUELEFT:
        firstPiece = blueLeftPiece;
        break;
      case BLUEMIDDLE:
        middle = true;
        break;
      case BLUERIGHT:
        firstPiece = blueLeftPiece;
        break;
    }

    if (!middle){
      command = new SequentialCommandGroup(
        new moveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),
        new AutoAlign(poseEstimator, lime, swerve),
<<<<<<< HEAD
        new moveTo(firstPiece.minus(poseEstimator.getPosition()), new Rotation2d(Math.PI), swerve, poseEstimator),
        new LimelightAlign(swerve, lime, 0, 0),
        new MoveArm(arm, claw, true, false, arm.ArmSetting.GNODE),
=======
        new moveTo(firstPiece.minus(poseEstimator.getPosition()), swerve, poseEstimator),
        new LimelightAlign(swerve, lime, VisionConstants.CubePipelineID, 0),
        
>>>>>>> a34a58cc50760cf43e730524ac1113ca8541704f
        new moveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),
        new AutoBalanceCommand(swerve)
      );
    }
    else {
      command = new SequentialCommandGroup(
        new moveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),
        new AutoAlign(poseEstimator, lime, swerve),
        new moveTo(firstPiece.minus(poseEstimator.getPosition()), new Rotation2d(Math.PI) swerve, poseEstimator),
        new LimelightAlign(swerve, lime, 0, 0),
        new MoveArm(arm, claw, true, false, arm.ArmSetting.GNODE),
        new moveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),
        new AutoAlign(poseEstimator, lime, swerve),
        new MoveArm(arm, claw, false, true, arm.ArmSetting.NODE3)
      );
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Move to Desired Station
    // ---- MoveTo, AutoAlign

    // Place Piece(Save for later)
    // ---- Needs Arm Commands

    // AprilTag align to find position of piece
    // ---- AprilTag, AutoAlign

    // Returning to Station
    // ---- MoveTo, AutoAlign

    // Placing Cone
    // ---- Arm Commands

    // If Balancing, Balance
    // ---- Balance
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
