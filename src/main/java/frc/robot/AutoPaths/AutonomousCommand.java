// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.AutoPaths;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.commands.AutoAlign;
// import frc.robot.commands.AutoBalanceCommand;
// import frc.robot.commands.LimelightAlign;
// import frc.robot.commands.MoveArm;
// import frc.robot.commands.MoveTo;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.PoseEstimation;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.ArmControlSubsystem.ArmSetting;
// import frc.robot.subsystems.ArmControlSubsystem;
// import frc.robot.subsystems.ClawMotor;



// public class AutonomousCommand extends CommandBase {
//   // UPDATE WITH POSES OF PIECES
//   public Pose2d redLeftPiece = new Pose2d();
//   public Pose2d redRightPiece = new Pose2d();
//   public Pose2d blueLeftPiece = new Pose2d();
//   public Pose2d blueRightPiece = new Pose2d();


//   public static enum paths {
//     REDLEFT,
//     REDRIGHT,
//     REDMIDDLE,
//     BLUELEFT,
//     BLUERIGHT,
//     BLUEMIDDLE
//   }

//   /** Creates a new AutonomousCommand. */
//   public Limelight lime;
//   public SwerveSubsystem swerve;
//   public MoveTo move;
//   public double desiredOffset = 12;
//   public boolean gotOffset;
//   public boolean movedForward;
//   public boolean turned;
//   public PoseEstimation poseEstimator;
//   public Pose2d firstPiece;
//   public ArmControlSubsystem arm;
//   public ClawMotor claw;
//   public SequentialCommandGroup command;


//   public AutonomousCommand(Limelight limeSub, SwerveSubsystem swerveSub, ArmControlSubsystem armSub, ClawMotor clawSub, PoseEstimation poseEstimatorr, paths path) {
//     lime = limeSub;
//     swerve = swerveSub;
//     poseEstimator = poseEstimatorr;
//     arm = armSub;
//     claw = clawSub;
//     addRequirements(swerve);
//     addRequirements(lime);
//     addRequirements(arm);
//     addRequirements(claw);
//     gotOffset = false;
//     movedForward = false;

//     boolean middle = false;
//     switch(path) {
//       case REDLEFT:
//         firstPiece = redLeftPiece;
//         break;
//       case REDRIGHT:
//         firstPiece = redRightPiece;
//         break;
//       case REDMIDDLE:
//         middle = true;
//         break;
//       case BLUELEFT:
//         firstPiece = blueLeftPiece;
//         break;
//       case BLUEMIDDLE:
//         middle = true;
//         break;
//       case BLUERIGHT:
//         firstPiece = blueLeftPiece;
//         break;
//     }
    

//     //assuming it starts facing towards positive X and that you are in line with the cube piece across
//     if (middle){
//       command = new SequentialCommandGroup(
//         //turn and lift arm
//         new ParallelCommandGroup(new MoveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),new MoveArm(arm, claw, false, false, false, ArmSetting.NODE3)),
//         //align with tag
//         new AutoAlign(poseEstimator, lime, swerve),
//         //spit out cube
//         new MoveArm(arm, claw, false, true, 1.0, ArmSetting.NODE3),
//         //move forward and lower arm
//         new MoveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),
//         new AutoBalanceCommand(swerveSub));    //     new ParallelCommandGroup(new MoveTo(new Transform2d(new Translation2d(5, 0), new Rotation2d(0)), swerve, poseEstimator),new MoveArm(arm, claw, false, false, false, ArmSetting.GNODE)),
//     //     //align with cube
//     //     new LimelightAlign(swerve, lime, 0, 0),
//     //     //move forward and get piece
//     //     new MoveTo(new Transform2d(new Translation2d(1, 0), new Rotation2d(0)), swerve, poseEstimator).raceWith(new MoveArm(arm, claw, true, false, true, ArmSetting.GNODE)),
//     //     //rotate
//     //     new MoveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),
//     //     //charge station
//     //     new AutoBalanceCommand(swerve)
//     // );
//     }
//     else {
//       //assuming it starts facing towards positive X and that you are in line with the cube piece across
//       command = new SequentialCommandGroup(
//         //turn and lift arm
//         new ParallelCommandGroup(new MoveTo(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),new MoveArm(arm, claw, false, false, false, ArmSetting.NODE3)),
//         // align with tag
//         new AutoAlign(poseEstimator, lime, swerve),
//         //spit out cube
//         new MoveArm(arm, claw, false, true, 1.0, ArmSetting.NODE3),
//         //move forward and lower arm
//         new ParallelCommandGroup(new MoveTo(new Transform2d(new Translation2d(3, 0), new Rotation2d(0)), swerve, poseEstimator),new MoveArm(arm, claw, false, false, false, ArmSetting.GNODE)),
//         //align with cube
//         new LimelightAlign(swerve, lime, 0, 0),
//         //move forward and get piece
//         new MoveTo(new Transform2d(new Translation2d(3, 0), new Rotation2d(0)), swerve, poseEstimator).raceWith(new MoveArm(arm, claw, true, false, true, ArmSetting.GNODE)),
//         //go back to nodes
//         new ParallelCommandGroup(new MoveTo(new Transform2d(new Translation2d(-5, 0), new Rotation2d(Math.PI)), swerve, poseEstimator),new MoveArm(arm, claw, false, false, false, ArmSetting.NODE3)),
//         //align with tag
//         new AutoAlign(poseEstimator, lime, swerve),
//         //spit out cube
//         new MoveArm(arm, claw, false, true, 1.0, ArmSetting.NODE3)
//       ); 
//     }
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     command.schedule();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Move to Desired Station
//     // ---- MoveTo, AutoAlign

//     // Place Piece(Save for later)
//     // ---- Needs Arm Commands

//     // AprilTag align to find position of piece
//     // ---- AprilTag, AutoAlign

//     // Returning to Station
//     // ---- MoveTo, AutoAlign

//     // Placing Cone
//     // ---- Arm Commands

//     // If Balancing, Balance
//     // ---- Balance
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return command.isFinished();
//   }
// }
