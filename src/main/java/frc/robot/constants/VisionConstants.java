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
    private VisionConstants(){
        throw new IllegalStateException("Constants Class");
      }
      
    public static final String CAMERA_NAME = "OV5647";

    // These are tunable constants for the reliability of odometry and vision measurements in the form of a vector of (x, y, theta), in meters, meters, and radians respectively
    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
    public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(7)); 

    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(.0496, .0626872, 1.28), new Rotation3d(0,-Math.toRadians(15),0));

    public static final double SINGLE_TAG_AMBIGUITY_THRESH = 0.15;
}