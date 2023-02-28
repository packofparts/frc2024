// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double pivotInitOffset = .5; //arbitrary. what the abs encoder returns when the arm is parallel to ground
    public static final double minAngleRad = Units.degreesToRadians(5.0) - Units.rotationsToRadians(pivotInitOffset) - Units.degreesToRadians(90);
    public static final double maxAngleRad = Units.degreesToRadians(115.0) - Units.rotationsToRadians(pivotInitOffset) - Units.degreesToRadians(90);
    public static final double[] angleLevelsDeg = {42.0-90, 100.976-90, 107.414-90}; //degrees

    public static final double extensionEncoderToLength = 1.0/10;
    public static final double minExtensionIn = 30; //basically the length of the first base //inches
    public static final double maxExtensionIn = 60;
    public static final double[] extensionLevelsIn = {36, 41, 60}; //inches


    //CAP af find this!!!
    public static final double pivotPosInMetersY = Units.inchesToMeters(45.75);


    //TBD
    public static final int rightArmPivot = 15;
    public static final int leftArmPivot = 16;
    public static final int telescopicArmSpark = 17;
    public static final int armPivotEncoderPort = 17;

    //Change this
    public static boolean leftPivotInverted = false;
}
