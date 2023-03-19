package frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;

public class CompConstants {

    
    public static final boolean debug = false; // Opens up ShuffleBoard Values

    // CAN Bus Reduction
    public static final boolean useAbsEncoder = false; // If we don't use the abs encoder, we don't even initialize them to reduce can traffic
    public static final boolean reduceRelativeFrameRate = false; // Reducing the relative encoder sampling rate
    public static final int reducedRelativeFrameRate = 50; // Relative encoder frame rate if we reduce IN MILLISECONDS


    public static final boolean kGyroHold = false; //if drift is too much

    // AutoBalance Tuning
    public static final PIDController velocityController = new PIDController(.45, .05, 0);
    public static final double onChargeStationOrientation = 15;
    public static final double entrySpeed = 1;
    public static float pitchSpeedThreshold = 35;

    public static final double kAngleDeadZoneDeg = 0.5;

    // Arm
    public static final boolean rateLimitArm = true;



}
