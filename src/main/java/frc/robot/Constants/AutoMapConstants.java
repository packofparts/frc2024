// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.armcontrolcmds.ExtensionCmd;
import frc.robot.commands.armcontrolcmds.PivotCmd;
import frc.robot.commands.armcontrolcmds.ScoreConeHighNode;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.PoseEstimation;
import frc.robot.vision.PoseEstimationBase;


// CODE ISN'T FUNCTIONAL YET


public class AutoMapConstants {

    public static enum GamePiece{
        CONE,
        CUBE
    }

    public static boolean doAction = true;
    public static double waitTime = 0.5;

    public static PathConstraints defaultSpeedConstrants = new PathConstraints(4, 3);
    public static PathConstraints slowSpeedConstraints = new PathConstraints(2, 1.5);

    public static PathPlannerTrajectory ConeCubeChargeBump = PathPlanner.loadPath("Cone+Ball+Charge", defaultSpeedConstrants);
    public static PathPlannerTrajectory ConeCubeBarrier = PathPlanner.loadPath("Cone+Cube Barrier", defaultSpeedConstrants);
    public static PathPlannerTrajectory ConeCubeBump = PathPlanner.loadPath("Cone+Cube Bump", defaultSpeedConstrants);

    
    
    public static PathPlannerTrajectory move1Meter = PathPlanner.loadPath("MoveOneMeters", defaultSpeedConstrants);
    public static PathPlannerTrajectory move1MeterRotate = PathPlanner.loadPath("MoveOneMeters+180", defaultSpeedConstrants);
    public static PathPlannerTrajectory backforth = PathPlanner.loadPath("backforth", defaultSpeedConstrants);

    public static PathPlannerTrajectory station2Piece = PathPlanner.loadPath("Station2Piece", defaultSpeedConstrants);
 
    public static HashMap<String,Command> eventMap = new HashMap<>();
    public static HashMap<String,Command> emptyMap =  new HashMap<>();

    public static void populateHashMaps(SwerveSubsystem swerve, LimelightPhoton lime, ArmControlSubsystem arm, PoseEstimationBase pose, ClawPnumatic claw){
        if (doAction){  

            eventMap.put("angle_N3", new PivotCmd(arm, ArmConstants.angleLevelsRad[2]));
            eventMap.put("angle_N2", new PivotCmd(arm, ArmConstants.angleLevelsRad[1]));
            eventMap.put("angle_N1", new PivotCmd(arm, ArmConstants.angleLevelsRad[0]));
            eventMap.put("angle_N0",new PivotCmd(arm, ArmConstants.groundPick[0]));
            eventMap.put("angle_neutral",new PivotCmd(arm, ArmConstants.angleLevelsRad[0]));

            eventMap.put("telescope_N3", new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[2]));
            eventMap.put("telescope_N2", new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[1]));
            eventMap.put("telescope_N1", new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[0]));
            eventMap.put("telescope_N0",new ExtensionCmd(arm, ArmConstants.groundPick[1]));
            eventMap.put("telescope_neutral",new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[0]));

            


            eventMap.put("auto_balance",new WaitCommand(1));
            eventMap.put("align_tag",new AutoAlign(pose, lime, swerve));

            eventMap.put("drop_cube", claw.dropPiece(GamePiece.CUBE));
            eventMap.put("intake_cube", new SequentialCommandGroup(new InstantCommand(()->claw.spinIntake(0.8)),new WaitCommand(0.8)));


            eventMap.put("auto_balance",new WaitCommand(1));
            //eventMap.put("align_cube", new LimelightAlign(swerve, lime, VisionConstants.CubePipelineID, LimelightAlign.MovementMode.ROTATE));
            eventMap.put("align_tag",new AutoAlign(pose, lime, swerve));


            //eventMap.put("score_cone", new ScoreConeHighNode(arm, claw));

            eventMap.put("score_cone", new SequentialCommandGroup(
                new PivotCmd(arm, ArmConstants.angleLevelsRad[2]),
                new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[2]),
                new WaitCommand(.5),
                claw.dropPiece(GamePiece.CONE)
            ));

            eventMap.put("stow_arm", new SequentialCommandGroup(
                new InstantCommand(() -> claw.closePneumatics(), claw),
                new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[0]),   
                new PivotCmd(arm, ArmConstants.angleLevelsRad[0])
            ));

            eventMap.put("stow_cube", new SequentialCommandGroup(
                new ExtensionCmd(arm, ArmConstants.extensionLevelsIn[0]),   
                new PivotCmd(arm, ArmConstants.angleLevelsRad[0]),
                new InstantCommand(() -> claw.setIntake(.2))
            ));
            
            eventMap.put("ground_pickup_cube", new SequentialCommandGroup(
                new PivotCmd(arm, ArmConstants.groundPick[0]),
                new InstantCommand(() -> claw.openPneumatics(), claw),
                new InstantCommand(() -> claw.setIntake(0.8), claw),
                new ExtensionCmd(arm, ArmConstants.groundPick[1])
            ));

        }else{
            
            eventMap.put("angle_N3", new WaitCommand(waitTime));
            eventMap.put("angle_N2", new WaitCommand(waitTime));
            eventMap.put("angle_N1", new WaitCommand(waitTime));
            eventMap.put("angle_N0", new WaitCommand(waitTime));
            eventMap.put("angle_neutral",new WaitCommand(waitTime));

            eventMap.put("telescope_N3", new WaitCommand(waitTime));
            eventMap.put("telescope_N2", new WaitCommand(waitTime));
            eventMap.put("telescope_N1", new WaitCommand(waitTime));
            eventMap.put("telescope_N0", new WaitCommand(waitTime));
            eventMap.put("telescope_neutral", new WaitCommand(waitTime));

            eventMap.put("score_cone",  new WaitCommand(waitTime));
            eventMap.put("drop_cube",  new WaitCommand(waitTime));
            eventMap.put("intake_cube",  new WaitCommand(waitTime));

            eventMap.put("auto_balance", new WaitCommand(waitTime));
            eventMap.put("align_cube",  new WaitCommand(waitTime));
            eventMap.put("align_tag", new WaitCommand(waitTime));
        }
    }
}

