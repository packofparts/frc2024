// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;

public class PIDConstants {
    public static final PIDController kFrontLeftSteeringPIDControl = 
        new PIDController(0.56, 0.0, 0.01);

    public static final PIDController kFrontRightSteeringPIDControl = 
        new PIDController(0.771, 0.025, 0.015);

    public static final PIDController kBackLeftSteeringPIDControl = 
        new PIDController(0.40, 0, 0.015);

    public static final PIDController kBackRightSteeringPIDControl = 
        new PIDController(0.56, 0, 0.01);

    public static final PIDController kFrontLeftDrivingMotorController = 
        new PIDController(0.5, 0, 0);

    public static final PIDController kFrontRightDrivingMotorController = 
        new PIDController(0.5, 0, 0);

    public static final PIDController kBackLeftDrivingMotorController = 
        new PIDController(0.5, 0, 0);

    public static final PIDController kBackRightDrivingMotorController = 
        new PIDController(0.5, 0, 0);

    public static PIDController transController = 
        new PIDController(0.4, 0, 0);

    public static PIDController XController = new PIDController(0.4, 0, 0);
    public static PIDController YController = new PIDController(0.4, 0, 0);
    public static PIDController rotController =  new PIDController(1.4, 0, 0);

    public static double[] transPIDValues = new double[]{0.7, 0, 0};
    public static double[] rotPIDValues = new double[]{2.2, 0, 0};
}
