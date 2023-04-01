// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    
    public static final double kPhysicalMaxSpeedMPS = 4.0;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
    public static final double kAutoMaxSpeedMPS = 3.0;
    public static final double kTeleMaxSpeedMPS = 4.0;


    public static double kTeleDriveMaxAccMPS = 5.0;
    public static double kTeleDriveMaxAngularAccRadPS = 4*Math.PI;
    public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = 8;
    public static double kDBHeightMeters = Units.inchesToMeters(3.75);

    public static double kAutoDriveMaxAccMPS = 3.0;
    public static double kAutoDriveMaxAngularAccRadPS = 2*Math.PI;
    public static double kAutoDriveMaxAngularSpeedRadiansPerSecond = 2;


    public static final double kDriveEncoderRot2Meter = Math.PI * Units.inchesToMeters(4); //multiply by gearratio check gear ratio for trans encoder
    public static final double RPMtoMPS = kDriveEncoderRot2Meter/60;


    //Encoders
    public static final double angleEncoderConversionFactortoRad = 2*Math.PI/18;
    public static final double driveEncoderConversionFactortoRotations = 1.0/10; 

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter*driveEncoderConversionFactortoRotations/ 60.0;

    public static final double kTrackWidthMeters = 0.54;
    public static double tuningSetpoint = 0;
    public static final double weirdAssOdVal = 0.266807902319739;
    
    // Distance between front and back wheels
    public static final double kWheelBase = 0.54;
    public static boolean tuningPID = false;

    
    // SPARK ids
    public static final int kFrontLeftDriveCANId = 1;
    public static final int kFrontLeftSteerCANId = 3;
        
    public static final int kFrontRightDriveCANId = 7;
    public static final int kFrontRightSteerCANId = 6;

    public static final int kBackLeftDriveCANId = 4;
    public static final int kBackLeftSteerCANId = 5;

    public static final int kBackRightDriveCANId = 9; 
    public static final int kBackRightSteerCANId = 8;

    // Encoder Offsets

    public static double kFrontLeftOffset = 0.480;
    public static double kFrontRightOffset = 0.731;
    public static double kBackLeftOffset = 0.775;
    public static double kBackRightOffset = 0.552;



    // CAN Traffic Reduction Booleans

}
