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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoMapConstants {
    public static PathPlannerTrajectory ConeCubeChargeTraj = PathPlanner.loadPath("Test Path", new PathConstraints(2, 1.5));
    public static HashMap<String,Command> m_EventMap =  new HashMap<>();

    public static void populateHashMaps(SwerveSubsystem swerve, Limelight lime, ClawPnumatic claw, ArmControlSubsystem arm){
        m_EventMap.put("angle_N3", new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.angleLevelsDeg[2]), arm));
        m_EventMap.put("angle_N2", new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.angleLevelsDeg[1]), arm));
        m_EventMap.put("angle_N1", new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.angleLevelsDeg[0]), arm));
        //m_EventMap.put("angle_N0",new InstantCommand(()))
        //Ball extension TBD
        m_EventMap.put("angle_neutral",new InstantCommand(()->arm.setDesiredPivotRotation(ArmConstants.minAngleRad), arm));

        m_EventMap.put("teloscope_N3", null);
        m_EventMap.put("teloscope_N2", null);
        m_EventMap.put("teloscope_N1", null);
        m_EventMap.put("teloscope_neutral",null);

        m_EventMap.put("auto_balance",null);
        m_EventMap.put("align_cube",null);
        m_EventMap.put("align_tag",null);

        m_EventMap.put("claw_open", null);
        m_EventMap.put("claw_close",null);
    }

}

