package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;

/** Add your docs here. */
public class ArmConstants {

    public static final boolean useAbsEncoderPiv = true;
    public static final boolean useAbsEncoderTelescope = false; 
    public static boolean useFeedForward = true;

    public static double 
    kG = 0.03;

    //channel on dio port
    public static final int DIOPortPiv = 0;

    public static final double pivotAbsEncToRotation = 1.0/3.7142857;
    public static double pivotInitOffset = 0;




    public static final double zeroAngleRad = Units.degreesToRadians(14);//11.5

    public static final double minAngleRad = Units.degreesToRadians(14); //33
    public static final double maxAngleRad = Units.degreesToRadians(115.0);

    public static final double extensionRotationToInches =  18.5/7.01;
    public static final double minExtensionIn = 0;
    //when it is at zeroAngleRad
    public static final double zeroExtensionIn = minExtensionIn;

    public static final double maxPivotRateRadSec = Units.degreesToRadians(70);
    public static final double physicalMaxPivotRadSec = Units.degreesToRadians(142);
    
    public static final double maxPivotRatePercentSec = maxPivotRateRadSec/physicalMaxPivotRadSec;

    public static final double maxExtensionIn = 18.5;

    // Setpoints
    public static final double gearRatioExtension = 1.0/10.0;

    public static final double[] extensionLevelsIn = {minExtensionIn, minExtensionIn, maxExtensionIn}; //inches
    public static final double[] angleLevelsDeg = {12, 87.0, 100}; //degrees
    public static final double[] angleLevelsRad = {Units.degreesToRadians(angleLevelsDeg[0]), Units.degreesToRadians(angleLevelsDeg[1]), Units.degreesToRadians(angleLevelsDeg[2])};
    public static final double[] groundPick  = {Units.degreesToRadians(40),9.5};

    public static final double[] offSubstation = {Units.degreesToRadians(90), 0}; // angle, inches including claw




    
    public static final double pivotPosInMetersY = Units.inchesToMeters(45.75);


    //TBD
    public static final int rightArmPivot = 16;
    public static final int leftArmPivot = 15;
    public static final int telescopicArmSpark = 10;
    public static final int armPivotEncoderPort = 17;

    public static final int extensionPort = 0;

    public static final double falconToFinalGear = 1.0/240;
    public static final double encoderResolution = 1.0/2048;
    public static final boolean kRateLimitArm = false;

    //Change this
    public static boolean leftPivotInverted = true;
}