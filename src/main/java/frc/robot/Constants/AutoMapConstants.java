// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.LimelightPhoton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.PoseEstimation;


// CODE ISN'T FUNCTIONAL YET

public class AutoMapConstants {
    public static PathPlannerTrajectory ConeCubeChargeTraj = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    public static HashMap<String,Command> m_EventMap =  new HashMap<>();

    public static void populateHashMaps(SwerveSubsystem swerve, LimelightPhoton lime, ArmControlSubsystem arm, PoseEstimation pose){
        m_EventMap.put("angle_N3", new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.angleLevelsDeg[2]), arm));
        m_EventMap.put("angle_N2", new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.angleLevelsDeg[1]), arm));
        m_EventMap.put("angle_N1", new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.angleLevelsDeg[0]), arm));
        //m_EventMap.put("angle_N0",new InstantCommand(()))
        //Ball extension TBD
        m_EventMap.put("angle_neutral",new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.minAngleRad), arm));

        m_EventMap.put("teloscope_N3", new InstantCommand(()->arm.setDesiredExtension(ArmConstants.extensionLevelsIn[2]), arm));
        m_EventMap.put("teloscope_N2", new InstantCommand(()->arm.setDesiredExtension(ArmConstants.extensionLevelsIn[1]), arm));
        m_EventMap.put("teloscope_N1", new InstantCommand(()->arm.setDesiredExtension(ArmConstants.extensionLevelsIn[0]), arm));
        m_EventMap.put("teloscope_neutral",new InstantCommand(()->arm.setDesiredExtension(ArmConstants.minExtensionIn), arm));

        m_EventMap.put("auto_balance",new AutoBalanceCommand(swerve));
        m_EventMap.put("align_cube", new LimelightAlign(swerve, lime, VisionConstants.CubePipelineID, 0));
        m_EventMap.put("align_tag",new AutoAlign(pose, lime, swerve));

    }

}

