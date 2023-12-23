// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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


        // Inverse Booleans
        private static final boolean FRONT_LEFT_ROT_INVERSE = false;
        private static final boolean FRONT_LEFT_TRANS_INVERSE = true;

        private static final boolean FRONT_RIGHT_ROT_INVERSE = false;
        private static final boolean FRONT_RIGHT_TRANS_INVERSE = false;

        private static final boolean BACK_LEFT_ROT_INVERSE = false;
        private static final boolean BACK_LEFT_TRANS_INVERSE = true;

        private static final boolean BACK_RIGHT_ROT_INVERSE = false;
        private static final boolean BACK_RIGHT_TRANS_INVERSE = false;

        // Swerve Module Locations
        public static final Translation2d FRONT_LEFT_COORDS_METERS =
                        new Translation2d(SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        SwerveConstants.TRACK_WIDTH_METERS / 2);
        public static final Translation2d FRONT_RIGHT_COORDS_METERS =
                        new Translation2d(SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        -SwerveConstants.TRACK_WIDTH_METERS / 2);
        public static final Translation2d BACK_LEFT_COORDS_METERS =
                        new Translation2d(-SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        SwerveConstants.TRACK_WIDTH_METERS / 2);
        public static final Translation2d BACK_RIGHT_COORDS_METERS =
                        new Translation2d(-SwerveConstants.TRACK_LENGTH_METERS / 2,
                                        -SwerveConstants.TRACK_WIDTH_METERS / 2);

        // PID Controllers
        public static final PIDController FRONT_LEFT_ROT_PID = new PIDController(0.35, 0., 0.);
        public static final PIDController FRONT_RIGHT_ROT_PID = new PIDController(0.35, 0.0, 0.);
        public static final PIDController BACK_RIGHT_ROT_PID = new PIDController(0.35, 0, 0.);
        public static final PIDController BACK_LEFT_ROT_PID = new PIDController(0.35, 0, 0);


        public static final double TRANS_P = 0.1;
        public static final double TRANS_I = 0.0;
        public static final double TRANS_D = 0.0;
        public static final double TRANS_FF = 0.238;


        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        FRONT_LEFT_COORDS_METERS, FRONT_RIGHT_COORDS_METERS,
                        BACK_LEFT_COORDS_METERS, BACK_RIGHT_COORDS_METERS);


        // Swerve Modules and Other Hardware
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
