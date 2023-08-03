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
        public static final int frontRightTransID = 0;
        public static final int frontRightRotID = 1;
        public static final int frontRightRotEncoderID = 2;

        public static final int frontLeftTransID = 3;
        public static final int frontLeftRotID = 4;
        public static final int frontLeftRotEncoderID = 5;

        public static final int backRightTransID = 6;
        public static final int backRightRotID = 7;
        public static final int backRightRotEncoderID = 8;

        public static final int backLeftRotID = 9;
        public static final int backLeftTransID = 10;
        public static final int backLeftRotEncoderID = 11;


        // Offsets

        // Inverse Booleans
        public static final boolean frontRightRotInverse = false;
        public static final boolean frontRightTransInverse = false;
        public static final boolean frontLeftRotInverse = false;
        public static final boolean frontLeftTransInverse = false;

        public static final boolean backRightRotInverse = false;
        public static final boolean backRightTransInverse = false;
        public static final boolean backLeftRotInverse = false;
        public static final boolean backLeftTransInverse = false;


        // PID Controllers
        public static final PIDController frontRightRotPID = new PIDController(0.5, 0, 0);
        public static final PIDController frontLeftRotPID = new PIDController(0.5, 0, 0);
        public static final PIDController backRightRotPID = new PIDController(0.5, 0, 0);
        public static final PIDController backLeftRotPID = new PIDController(0.5, 0, 0);


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

        public static final AHRS navX = new AHRS(Port.kMXP);

        public static final SwerveModule frontRightModule = new SwerveModule(frontRightRotID,
                        frontRightTransID, frontRightRotEncoderID,
                        SwerveConstants.kfrontRightRotEncoderOffset, frontRightRotInverse,
                        frontRightTransInverse, frontRightRotPID);

        public static final SwerveModule frontLeftModule = new SwerveModule(frontLeftRotID,
                        frontLeftTransID, frontLeftRotEncoderID,
                        SwerveConstants.kfrontLeftRotEncoderOffset, frontLeftRotInverse,
                        frontLeftTransInverse, frontLeftRotPID);

        public static final SwerveModule backRightModule = new SwerveModule(backRightRotID,
                        backRightTransID, backRightRotEncoderID,
                        SwerveConstants.kbackRightRotEncoderOffset, backRightRotInverse,
                        backRightTransInverse, backRightRotPID);

        public static final SwerveModule backLeftModule =
                        new SwerveModule(backLeftRotID, backLeftTransID, backLeftRotEncoderID,
                                        SwerveConstants.kbackLeftRotEncoderOffset,
                                        backLeftRotInverse, backLeftTransInverse, backLeftRotPID);

        public static final SwerveModule[] swerveModules =
                        {frontRightModule, frontLeftModule, backRightModule, backLeftModule};

}
