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
        private SwerveConfig() {
                throw new IllegalStateException("Static Config Class");
        }

        // ID's
        // Encoder IDs have been set
        private static final int FRONT_LEFT_TRANS_ID = 1;
        private static final int FRONT_LEFT_ROT_ID = 8;
        private static final int FRONT_LEFT_ROT_ENC_ID = 22;

        private static final int FRONT_RIGHT_TRANS_ID = 7;
        private static final int FRONT_RIGHT_ROT_ID = 3;
        private static final int FRONT_RIGHT_ROT_ENC_ID = 23;

        private static final int BACK_RIGHT_TRANS_ID = 9;
        private static final int BACK_RIGHT_ROT_ID = 6;
        private static final int BACK_RIGHT_ROT_ENC_ID = 20;

        private static final int BACK_LEFT_TRANS_ID = 5;
        private static final int BACK_LEFT_ROT_ID = 4;
        private static final int BACK_LEFT_ROT_ENC_ID = 21;


        // Offsets

        // Inverse Booleans
        private static final boolean FRONT_LEFT_ROT_INVERSE = false;
        private static final boolean FRONT_LEFT_TRANS_INVERSE = true;

        private static final boolean FRONT_RIGHT_ROT_INVERSE = false;
        private static final boolean FRONT_RIGHT_TRANS_INVERSE = false;

        private static final boolean BACK_LEFT_ROT_INVERSE = false;
        private static final boolean BACK_LEFT_TRANS_INVERSE = true;

        private static final boolean BACK_RIGHT_ROT_INVERSE = false;
        private static final boolean BACK_RIGHT_TRANS_INVERSE = false;



        // PID Controllers
        public static final PIDController FRONT_LEFT_ROT_PID = new PIDController(0.35, 0., 0.);
        public static final PIDController FRONT_RIGHT_ROT_PID = new PIDController(0.35, 0.0, 0.);
        public static final PIDController BACK_RIGHT_ROT_PID = new PIDController(0.35, 0, 0.);
        public static final PIDController BACK_LEFT_ROT_PID = new PIDController(0.35, 0, 0);


        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        SwerveConstants.TRACK_WIDTH_METERS / 2),
                        new Translation2d(SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        -SwerveConstants.TRACK_WIDTH_METERS / 2),
                        new Translation2d(-SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        SwerveConstants.TRACK_WIDTH_METERS / 2),
                        new Translation2d(-SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        -SwerveConstants.TRACK_WIDTH_METERS / 2));


        // Swerve Modules and Other Hardware

        public static final AHRS NAVX = new AHRS(Port.kMXP);

        public static final SwerveModule FRONT_LEFT_MODULE = new SwerveModule(FRONT_LEFT_ROT_ID,
                        FRONT_LEFT_TRANS_ID, FRONT_LEFT_ROT_ENC_ID, FRONT_LEFT_ROT_INVERSE,
                        FRONT_LEFT_TRANS_INVERSE, FRONT_LEFT_ROT_PID);

        public static final SwerveModule FRONT_RIGHT_MODULE = new SwerveModule(FRONT_RIGHT_ROT_ID,
                        FRONT_RIGHT_TRANS_ID, FRONT_RIGHT_ROT_ENC_ID, FRONT_RIGHT_ROT_INVERSE,
                        FRONT_RIGHT_TRANS_INVERSE, FRONT_RIGHT_ROT_PID);

        public static final SwerveModule BACK_RIGHT_MODULE = new SwerveModule(BACK_RIGHT_ROT_ID,
                        BACK_RIGHT_TRANS_ID, BACK_RIGHT_ROT_ENC_ID, BACK_RIGHT_ROT_INVERSE,
                        BACK_RIGHT_TRANS_INVERSE, BACK_RIGHT_ROT_PID);

        public static final SwerveModule BACK_LEFT_MODULE = new SwerveModule(BACK_LEFT_ROT_ID,
                        BACK_LEFT_TRANS_ID, BACK_LEFT_ROT_ENC_ID, BACK_LEFT_ROT_INVERSE,
                        BACK_LEFT_TRANS_INVERSE, BACK_LEFT_ROT_PID);

        public static final SwerveModule[] SWERVE_MODULES = {FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE,
                        BACK_LEFT_MODULE, BACK_RIGHT_MODULE};

        public static final PIDController[] SWERVE_MODULE_PIDs = {FRONT_LEFT_ROT_PID,
                        FRONT_RIGHT_ROT_PID, BACK_LEFT_ROT_PID, BACK_LEFT_ROT_PID};

}
