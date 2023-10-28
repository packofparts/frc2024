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

/** Add your docs here. */
public class SwerveConfig {
        private SwerveConfig(){
                throw new IllegalStateException("Static Config Class");
        }
        // ID's
        //Encoder IDs have been set
        private static final int kfrontRightTransID = 7;
        private static final int kfrontRightRotID = 3;
        private static final int kfrontRightRotEncoderID = 23;

        private static final int kfrontLeftTransID = 1;
        private static final int kfrontLeftRotID = 8;
        private static final int kfrontLeftRotEncoderID = 22;

        private static final int kbackRightTransID = 9;
        private static final int kbackRightRotID = 6;
        private static final int kbackRightRotEncoderID = 20;

        private static final int kbackLeftTransID = 5;
        private static final int kbackLeftRotID = 4;
        private static final int kbackLeftRotEncoderID = 21;


        // Offsets

        // Inverse Booleans
        private static final boolean kfrontRightRotInverse = false;
        private static final boolean kfrontRightTransInverse = false;
        private static final boolean kfrontLeftRotInverse = false;
        private static final boolean kfrontLeftTransInverse = true;

        private static final boolean kbackRightRotInverse = false;
        private static final boolean kbackRightTransInverse = false;
        private static final boolean kbackLeftRotInverse = false;
        private static final boolean kbackLeftTransInverse = true;


        // PID Controllers
        public static final PIDController frontRightRotPID = new PIDController(0.35, 0.0, 0.);
        public static final PIDController frontLeftRotPID = new PIDController(0.35, 0., 0.);
        public static final PIDController backRightRotPID = new PIDController(0.35, 0, 0.);
        public static final PIDController backLeftRotPID = new PIDController(0.35, 0, 0);


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

        public static final SwerveModule frontRightModule = new SwerveModule(kfrontRightRotID,
                        kfrontRightTransID, kfrontRightRotEncoderID,
                        kfrontRightRotInverse,
                        kfrontRightTransInverse, frontRightRotPID);

        public static final SwerveModule frontLeftModule = new SwerveModule(kfrontLeftRotID,
                        kfrontLeftTransID, kfrontLeftRotEncoderID,
                        kfrontLeftRotInverse,
                        kfrontLeftTransInverse, frontLeftRotPID);

        public static final SwerveModule backRightModule = new SwerveModule(kbackRightRotID,
                        kbackRightTransID, kbackRightRotEncoderID,
                        kbackRightRotInverse,
                        kbackRightTransInverse, backRightRotPID);

        public static final SwerveModule backLeftModule = new SwerveModule(kbackLeftRotID,
                        kbackLeftTransID, kbackLeftRotEncoderID,
                        kbackLeftRotInverse, 
                        kbackLeftTransInverse, backLeftRotPID);

        public static final SwerveModule[] swerveModules =
                        {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        public static final PIDController[] swerveModulePIDs =
                        {frontLeftRotPID, frontRightRotPID, backLeftRotPID, backLeftRotPID};

}
