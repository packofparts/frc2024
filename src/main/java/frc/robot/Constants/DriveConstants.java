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


    public static final double kDriveEncoderRot2Meter = 2*Math.PI * Units.inchesToMeters(2); //multiply by gearratio check gear ratio for trans encoder
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

    
    //SPARK ids
    public static final int frontLeftSteer = 3;//
    public static final int frontLeftDrive = 1;//
        
    public static final int frontRightSteer = 6;//
    public static final int frontRightDrive = 7;//

    public static final int rearLeftSteer = 5;//
    public static final int rearLeftDrive = 4;//

    public static final int rearRightSteer = 9;//
    public static final int rearRightDrive = 8; // 

    public static final boolean gyroHold = false;

}
