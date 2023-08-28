package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public enum SwerveConfig {
    FRONT_LEFT(
        /** module name */ "FRONT_LEFT",
        /** drive motor CAN ID */ 1, 
        /** drive motor inverse */ false, 

        /** steering motor CAN ID */ 8, 
        /** steering motor inverse */ false, 

        /** steering encoder CAN ID */ 22, 
        /** steering encoder offset radians */ Const.kFrontLeftRotationEncoderOffsetRad,

        /** driving PID controller */ new PIDController(1, 0, 0),
        /** steering PID controller */ new PIDController(1, 0, 0),

        /** location */ Const.kFrontLeftLocation), 

    FRONT_RIGHT(
        /** module name */ "FRONT_RIGHT",
        /** drive motor CAN ID */ 7, 
        /** drive motor inverse */ true,

        /** steering motor CAN ID */ 3,
        /** steering motor inverse */ false, 

        /** steering encoder CAN ID */ 23, 
        /** steering encoder offset radians */ Const.kFrontRightRotationEncoderOffsetRad,

        /** driving PID controller */ new PIDController(1, 0, 0),
        /** steering PID controller */ new PIDController(1, 0, 0),

        /** location */ Const.kFrontRightLocation),
        
    BACK_LEFT(
        /** module name */ "BACK_LEFT",
        /** drive motor CAN ID */ 5,
        /** drive motor inverse */ false, 

        /** steering motor CAN ID */ 4,
        /** steering motor inverse */ false, 

        /** steering encoder CAN ID */ 21, 
        /** steering encoder offset radians */ Const.kBackLeftRotationEncoderOffsetRad,

        /** driving PID controller */ new PIDController(1, 0, 0),
        /** steering PID controller */ new PIDController(1, 0, 0),

        /** location */ Const.kBackLeftLocation),        

    BACK_RIGHT(
        /** module name */ "BACK_RIGHT",
        /** drive motor CAN ID */ 9, 
        /** drive motor inverse */ true,

        /** steering motor CAN ID */ 6,
        /** steering motor inverse */ false, 

        /** steering encoder CAN ID */ 20, 
        /** steering encoder offset radians */ Const.kBackRightRotationEncoderOffsetRad,

        /** driving PID controller */ new PIDController(1, 0, 0),
        /** steering PID controller */ new PIDController(1, 0, 0),

        /** location */ Const.kBackRightLocation);
    
    public final String moduleName;        
    public final int driveMotorCanID;
    public final boolean driveMotorInverse;

    public final int rotationMotorCanID;
    public final boolean rotationMotorInverse;

    public final int rotationEncoderCanID;
    public final double rotationEncoderOffsetDegrees;

    public final PIDController drivePIDController;
    public final PIDController rotationPIDController;

    public final Translation2d location;

    SwerveConfig(
        String moduleName,
        int driveMotorCanID,
        boolean driveMotorInverse,

        int rotationMotorCanID,
        boolean rotationMotorInverse,
        
        int rotationEncoderCanID,
        double rotationEncoderOffsetDegrees,

        PIDController drivePIDController, 
        PIDController rotationPIDController,
        Translation2d location) {

        this.moduleName = moduleName;
        this.driveMotorCanID = driveMotorCanID;
        this.driveMotorInverse = driveMotorInverse;
        
        this.rotationMotorCanID = rotationMotorCanID;
        this.rotationMotorInverse = rotationMotorInverse;

        this.rotationEncoderCanID = rotationEncoderCanID;
        this.rotationEncoderOffsetDegrees = rotationEncoderOffsetDegrees;

        this.drivePIDController = drivePIDController;
        this.rotationPIDController = rotationPIDController;

        this.location = location;
    }

    static class Const {
        private Const() {}
        
        // TODO check the next 3 constants 
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelBaseMeters = Units.inchesToMeters(18.5);
        public static final double kTrackWidthMeters = Units.inchesToMeters(18.5);

        public static final Translation2d kFrontLeftLocation = new Translation2d(kWheelBaseMeters/2, kTrackWidthMeters/2);
        public static final Translation2d kFrontRightLocation = new Translation2d(kWheelBaseMeters/2, -kTrackWidthMeters/2);
        public static final Translation2d kBackLeftLocation = new Translation2d(-kWheelBaseMeters/2, kTrackWidthMeters/2);
        public static final Translation2d kBackRightLocation = new Translation2d(-kWheelBaseMeters/2, -kTrackWidthMeters/2);
        
        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            kFrontLeftLocation,
            kFrontRightLocation,
            kBackLeftLocation,
            kBackRightLocation);

        // 1/2 rotation per second
        public static final double kMaxAngularSpeedRadiansPerSec = Math.PI; 
        
        // TODO check of these constants are correct and the range of values are valid
        public static final double kFrontLeftRotationEncoderOffsetRad = (0.966-0.5)*Math.PI;
        public static final double kFrontRightRotationEncoderOffsetRad = (0.228+0.5-0.79)*Math.PI;
    
        public static final double kBackLeftRotationEncoderOffsetRad = (0.699)*Math.PI;
        public static final double kBackRightRotationEncoderOffsetRad = (0.721)*Math.PI;
    }    
}
