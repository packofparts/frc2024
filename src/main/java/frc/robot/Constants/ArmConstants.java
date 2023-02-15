// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double pivotInitOffset = .5; //arbitrary. what the abs encoder returns when the arm is parallel to ground
    public static final double minAngle = Units.degreesToRadians(5.0) - Units.rotationsToRadians(pivotInitOffset) - Units.degreesToRadians(90);
    public static final double maxAngle = Units.degreesToRadians(115.0) - Units.rotationsToRadians(pivotInitOffset) - Units.degreesToRadians(90);
    public static final double[] angleLevels = {42.0-90, 100.976-90, 107.414-90}; //degrees

    public static final double extensionEncoderToLength = 1.0/10;
    public static final double minExtension = 30; //basically the length of the first base //inches
    public static final double maxExtension = 60;
    public static final double[] extensionLevels = {36, 41, 60}; //inches


    public static final double pivotPosInMetersY = 4;

    public static final int rightArmPivot = 15;
    public static final int leftArmPivot = 16;
    public static final int telescopicArmSpark = 17;

    public static final int armPivotEncoderPort = 17;
}
