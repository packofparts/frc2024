// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
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

    public static final double kTrackWidth = 0.54;
    public static double tuningSetpoint = 0;
    public static final double weirdAssOdVal = 0.266807902319739;
    
    // Distance between front and back wheels
    public static final double kWheelBase = 0.54;
    public static boolean tuningPID = false;
    
    //SPARK ids
    public static final int frontLeftSteer = 1;//
    public static final int frontLeftDrive = 3;//
        
    public static final int frontRightSteer = 6;//
    public static final int frontRightDrive = 7;//

    public static final int rearLeftSteer = 5;//
    public static final int rearLeftDrive = 4;//

    public static final int rearRightSteer = 9;//
    public static final int rearRightDrive = 8; // 

    public static final boolean gyroHold = false;

}
