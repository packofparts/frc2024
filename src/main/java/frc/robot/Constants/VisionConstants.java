// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static final int CubePipelineID = 0;
    public static final int TagPiplineID = 1;
    public static final int ReflectivePipelineID = 2;

    //Left
    public static final double camXOffsetMeters = -0.18;

    //Up
    public static final double camYOffsetMeters = 0.62;

    //CCW
    public static final double camPitchOffsetDegrees = -15;

    public static final Transform3d robotToCam = new Transform3d(new Translation3d(camXOffsetMeters, camYOffsetMeters, 0),

    new Rotation3d(0, Units.degreesToRadians(camPitchOffsetDegrees),0));
    
    //TODO
    public static final double LimelightConstantOffset = 2.617;


    

    // Pose Estimation Constants
    
    public static final int maxDistance = 2; // Maximum distance for vision to update in meters
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)); 
    
    //TODO CONSTANTS
    public static final Transform2d substationAlign = new Transform2d();
    public static final Transform2d autoAlign = new Transform2d();
    
}
