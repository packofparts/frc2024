// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double pivotInitOffset = 0; //arbitrary. what the abs encoder returns when the arm is parallel to ground
    public static final double minAngleRad = Units.degreesToRadians(5.0) - Units.rotationsToRadians(pivotInitOffset) - Units.degreesToRadians(90);
    public static final double maxAngleRad = Units.degreesToRadians(115.0) - Units.rotationsToRadians(pivotInitOffset) - Units.degreesToRadians(90);

    public static final double extensionEncoderToLength =  1.0/8.11330126;
    public static final double minExtensionIn = 29.85 + 7.073; //basically the length of the first base //inches
    public static final double maxExtensionIn = 48.85 + 7.073;
    public static final double armToRobotAngle = 43.288;



    // Setpoints
    public static final double[] extensionLevelsIn = {minExtensionIn, minExtensionIn, 43.0}; //inches
    public static final double[] angleLevelsDeg = {35.0-90, 85.0-90, 99.75-90}; //degrees

    public static final double[] offSubstation = {85.5-90, 46.0}; // angle, inches including claw
//    public static final double[] offGround



    //CAP af find this!!!
    public static final double pivotPosInMetersY = Units.inchesToMeters(45.75);


    //TBD
    public static final int rightArmPivot = 16;
    public static final int leftArmPivot = 15;
    public static final int telescopicArmSpark = 17;
    public static final int armPivotEncoderPort = 17;
    public static final int clawPort = 0;

    public static final double relEncoderToInitialGear = 1.0/48;

    //Change this
    public static boolean leftPivotInverted = false;
}
