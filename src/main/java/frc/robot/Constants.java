/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
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

    public static final double rad2Deg = 180/Math.PI;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    public static final double kDriveEncoderRot2Meter = 2*Math.PI * Units.inchesToMeters(4)/10;
    public static final double kDriveGearRation = 1/10;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter*kDriveGearRation / 60;

    //Encoders
    public static final double angleEncoderConversionFactor = 2*Math.PI/18;
    public static final double driveEncoderConversionFactor = 1;

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
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    public static double tuningSetpoint = 0;
    
    // // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final boolean tuningPID = false;
    
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

    //TEJA IS COOL

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


    //Gyro Adjustments
    public static double priorGyroAngle = 0;
}
