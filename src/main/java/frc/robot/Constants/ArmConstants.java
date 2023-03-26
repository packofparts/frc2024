package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static final double pivotInitOffset = 0; //arbitrary. what the abs encoder returns when the arm is parallel to ground
    public static final double zeroAngleRad = Units.degreesToRadians(12);//11.5

    public static final double minAngleRad = Units.degreesToRadians(12); //33
    public static final double maxAngleRad = Units.degreesToRadians(115.0);

    public static final double extensionRotationToInches =  20/5.96533203125;
    public static final double minExtensionIn = 0;//29.85+ 7.073; //basically the length of the first base //inches
    //when it is at zeroAngleRad
    public static final double zeroExtensionIn = .5 + minExtensionIn;

    public static final double maxPivotRateRadSec = Units.degreesToRadians(70);
    public static final double pysicalMaxPivotRadSec = Units.degreesToRadians(142);
    
    public static final double maxPivotRatePercentSec = maxPivotRateRadSec/pysicalMaxPivotRadSec;

    public static final double maxExtensionIn = 20;

    // Setpoints
    public static final double gearRatioExtension = 1.0/10;

    public static final double[] extensionLevelsIn = {minExtensionIn, minExtensionIn, maxExtensionIn}; //inches
    public static final double[] angleLevelsDeg = {12, 87.0, 97}; //degrees
    public static final double groundPick  = 33;

    public static final double[] offSubstation = {91, 0}; // angle, inches including claw




    
    public static final double pivotPosInMetersY = Units.inchesToMeters(45.75);


    //TBD
    public static final int rightArmPivot = 16;
    public static final int leftArmPivot = 15;
    public static final int telescopicArmSpark = 10;
    public static final int armPivotEncoderPort = 17;

    public static final int extensionPort = 0;

    public static final double falconToFinalGear = 1.0/240;
    public static final double encoderResolution = 1.0/2048;
    //Change this
    public static boolean leftPivotInverted = true;
}