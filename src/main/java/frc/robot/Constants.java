/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //SWERVE constants    
    public static final double kMaxSpeedMPS = 4.0;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2.0 * 2.0 * Math.PI;
    public static final double rad2Deg = 180/Math.PI;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 1.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    public static final double kDriveEncoderRot2Meter = 2*Math.PI * Units.inchesToMeters(2); //multiply by gearratio check gear ratio for trans encoder
    public static final double RPMtoMPS = kDriveEncoderRot2Meter/60;


    //Encoders
    public static final double angleEncoderConversionFactortoRad = 2*Math.PI/18;
    public static final double driveEncoderConversionFactortoRotations = 1.0/10; 

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter*driveEncoderConversionFactortoRotations/ 60.0;

    //Limelight Pipelines
    public static final int CubePipelineID = 0;
    public static final int TagPiplineID = 1;
    public static final int ReflectivePipelineID = 2;


    //PID
    public static final PIDController frPID = new PIDController(0.771, 0.025, 0.015);
    public static final PIDController flPID = new PIDController(0.56, 0.025, 0.01);
    public static final PIDController brPID = new PIDController(0.771, 0, 0.015);
    public static final PIDController blPID = new PIDController(0.56, 0, 0.01);

    public static final PIDController frPIDTrans = new PIDController(0.5, 0, 0);
    public static final PIDController flPIDTrans = new PIDController(0.5, 0, 0);
    public static final PIDController brPIDTrans = new PIDController(0.5, 0, 0);
    public static final PIDController blPIDTrans = new PIDController(0.5, 0, 0);

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(19.8819);
    public static double tuningSetpoint = 0;
    public static final double weirdAssOdVal = 0.266807902319739;
    
    // // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static boolean tuningPID = false;
    
    //SPARK ids
    public static final int frontLeftSteer = 3;//
    public static final int frontLeftDrive = 2;//
        
    public static final int frontRightSteer = 5;//
    public static final int frontRightDrive = 4;//

    public static final int rearLeftSteer = 6;//
    public static final int rearLeftDrive = 7;//

    public static final int rearRightSteer = 8;//
    public static final int rearRightDrive = 9; // 

    public static final boolean gyroHold = false;

    //TEJA IS THE BIGGEST BIRD

    //Change this
    public static int rotJoystickPort = 0;
    public static int transJoystickPort = 1;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static int intakeSolenoid1ID;
    public static int compressorID;


    public static double deadZone = 0.2;
    public static double radDeadZone = 0.05;

    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    //arm stuff
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


    public static final double kRotP = 0.005;

    //Gyro Adjustments
    public static double priorGyroAngle = 0;

    public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));


    //Constants For Charge Station:
    public static final double kChargeStationFullLengthCm = 193;
    public static final double kChargeStationBalanceBeamLengthCm = 122;
    public static final double kChargeStationMountLengthCm = 42.2995271841; 
    public static final double kAngleDeadZoneDeg = 0.5;


    //Constants for TGWithPPlib following:
    public static final HashMap <String,Command> eventMap = new HashMap<>();
    //empty for now
}
