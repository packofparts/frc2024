/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.util.Units;

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
    public static final double maxSpeed = 12.0;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    public static final double kDriveEncoderRot2Meter = Math.PI * Units.inchesToMeters(4);
    public static final double kDriveGearRation = 1/10;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter*kDriveGearRation / 60;

    //Encoders
    public static final double angleEncoderConversionFactor = 2*Math.PI/18;
    public static final double driveEncoderConversionFactor = 1;

    //PID
    public static final Gains anglePID = new Gains(0.005, 0, 0, 0.0, 0.0, -0.5, 0.5, 0);
    public static final Gains anglePIDFast = new Gains(0.005, 0, 0, 0.0, 0.0, -1, 1, 1);
    public static final Gains fastPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -1, 1, 1);

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    public static double tuningSetpoint = 0;
    
    // // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final Boolean tuningPID = true;
    
    //SPARK ids
    public static final int frontLeftSteer = 14;//
    public static final int frontLeftDrive = 13;//
    
    public static final int frontRightSteer = 2;//
    public static final int frontRightDrive = 4;//

    public static final int rearLeftSteer = 3;//
    public static final int rearLeftDrive = 12;//

    public static final int rearRightSteer = 1;//
    public static final int rearRightDrive = 5; // 

    //TEJA IS COOL

    //Change this
    public static int rotJoystickPort = 0;
    public static int transJoystickPort = 1;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static int intakeSolenoid1ID;
    public static int compressorID;


    public static double deadZone = 0.25;
    public static double radDeadZone = 0.05;
}
