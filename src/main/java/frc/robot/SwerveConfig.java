// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class SwerveConfig {
        // ID's
        public static int frontRightTransID = 0;
        public static int frontRightRotID = 1;
        public static int frontRightRotEncoderID = 2;

        public static int frontLeftTransID = 3;
        public static int frontLeftRotID = 4;
        public static int frontLeftRotEncoderID = 5;

        public static int backRightTransID = 6;
        public static int backRightRotID = 7;
        public static int backRightRotEncoderID = 8;

        public static int backLeftRotID = 9;
        public static int backLeftTransID = 10;
        public static int backLeftRotEncoderID = 11;


        // Offsets

        // Inverse Booleans
        public static boolean frontRightRotInverse = false;
        public static boolean frontRightTransInverse = false;
        public static boolean frontLeftRotInverse = false;
        public static boolean frontLeftTransInverse = false;

        public static boolean backRightRotInverse = false;
        public static boolean backRightTransInverse = false;
        public static boolean backLeftRotInverse = false;
        public static boolean backLeftTransInverse = false;


        // PID Controllers
        public static PIDController frontRightRotPID = new PIDController(0.5, 0, 0);
        public static PIDController frontLeftRotPID = new PIDController(0.5, 0, 0);
        public static PIDController backRightRotPID = new PIDController(0.5, 0, 0);
        public static PIDController backLeftRotPID = new PIDController(0.5, 0, 0);


        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                        new Translation2d(SwerveConstants.kTrackLengthMeters / 2,
                                        SwerveConstants.kTrackWidthMeters / 2),
                        new Translation2d(SwerveConstants.kTrackLengthMeters / 2,
                                        -SwerveConstants.kTrackWidthMeters / 2),
                        new Translation2d(-SwerveConstants.kTrackLengthMeters / 2,
                                        SwerveConstants.kTrackWidthMeters / 2),
                        new Translation2d(-SwerveConstants.kTrackLengthMeters / 2,
                                        -SwerveConstants.kTrackWidthMeters / 2));


        // Swerve Modules and Other Hardware

        public static AHRS navX = new AHRS(Port.kMXP);

        public static SwerveModule frontRightModule = new SwerveModule(frontRightRotID,
                        frontRightTransID, frontRightRotEncoderID,
                        SwerveConstants.frontRightRotEncoderOffset, frontRightRotInverse,
                        frontRightTransInverse, frontRightRotPID);

        public static SwerveModule frontLeftModule = new SwerveModule(frontLeftRotID,
                        frontLeftTransID, frontLeftRotEncoderID,
                        SwerveConstants.frontLeftRotEncoderOffset, frontLeftRotInverse,
                        frontLeftTransInverse, frontLeftRotPID);

        public static SwerveModule backRightModule = new SwerveModule(backRightRotID,
                        backRightTransID, backRightRotEncoderID,
                        SwerveConstants.backRightRotEncoderOffset, backRightRotInverse,
                        backRightTransInverse, backRightRotPID);

        public static SwerveModule backLeftModule = new SwerveModule(backLeftRotID, backLeftTransID,
                        backLeftRotEncoderID, SwerveConstants.backLeftRotEncoderOffset,
                        backLeftRotInverse, backLeftTransInverse, backLeftRotPID);

        public static SwerveModule[] swerveModules =
                        {frontRightModule, frontLeftModule, backRightModule, backLeftModule};

}
