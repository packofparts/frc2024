package frc.robot.Constants;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;

/** Add your docs here. */
public class ArmConstants {

    public static final boolean useAbsEncoderPiv = false;
    public static final boolean useAbsEncoderTelescope = false; 


    //channel on dio port
    public static final int DIOPortPiv = 0;

    public static final double pivotAbsEncToRotation = 0;
    public static double pivotInitOffset = 0; //arbitrary. what the abs encoder returns when the arm is parallel to ground




    public static final double zeroAngleRad = Units.degreesToRadians(11.5);//11.5

    public static final double minAngleRad = Units.degreesToRadians(30); //33
    public static final double maxAngleRad = Units.degreesToRadians(115.0);

    public static final double extensionRotationToInches =  18.3/5.96533203125;
    public static final double minExtensionIn = 0;//29.85+ 7.073; //basically the length of the first base //inches
    //when it is at zeroAngleRad
    public static final double zeroExtensionIn = 1.618 + minExtensionIn;

    public static final double maxPivotRateRadSec = Units.degreesToRadians(70);
    public static final double pysicalMaxPivotRadSec = Units.degreesToRadians(142);
    
    public static final double maxPivotRatePercentSec = maxPivotRateRadSec/pysicalMaxPivotRadSec;

    public static final double maxExtensionIn = 18;

    // Setpoints
    public static final double gearRatioExtension = 1.0/10;

    public static final double[] extensionLevelsIn = {minExtensionIn, minExtensionIn, maxExtensionIn}; //inches
    public static final double[] angleLevelsDeg = {35.0, 75.0, 99.75}; //degrees

    public static final double[] offSubstation = {92, 7}; // angle, inches including claw
//    public static final double[] offGround



    //CAP af find this!!!
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