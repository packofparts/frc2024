// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static final String kCameraName = "OV5647";

    // These are tunable constants for the reliability of odometry and vision measurements in the form of a vector of (x, y, theta), in meters, meters, and radians respectively
    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(1));
    public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(10)); 
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.1, 0.05, 1.24), new Rotation3d(0,0,0));

    public static final double kSingleTagAmbiguityThreshold = 0.1;
}