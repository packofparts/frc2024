package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;

/** Add your docs here. */
public class ArmConstants {

    public static enum ArmState {
        UNDEFINED(10000,10000),
        STOW(minAngleRad,minExtensionIn),
        GROUND_PICKUP(Units.degreesToRadians(40),9.5),
        LOWER_NODE(Units.degreesToRadians(12),0),
        MID_NODE(Units.degreesToRadians(87.0),0),
        UPPER_NODE(Units.degreesToRadians(100),17.1),
        SUBSTATION(Units.degreesToRadians(90),0),
        GROUND_PICKUP_CONE(Units.degreesToRadians(47),16.7);
    
        public double pivotAngleRad;
        public double extentionDistIn;
        ArmState(double piv, double ext){
          this.pivotAngleRad = piv;
          this.extentionDistIn = ext;
        }
      }
        
    public static ArmState curArmState = ArmState.STOW;
    public static double armStatePivDeadzoneRad = Units.degreesToRadians(3);
    public static double armStateExtDeadzoneIn = 2; 

    public static boolean useFeedForward = true;
    public static double kG = 0.03;

    //channel on dio port
    public static final int DIOPortPiv = 0;
    public static final double pivotAbsEncToRotation = 1.0/3.7142857;
    public static final double extensionRotationToInches =  18.5/7.01;



    // Offset For Pivot
    public static double pivotInitOffset = 0.053+0.025;
    
    //Initial Value
    public static final double zeroAngleRad = Units.degreesToRadians(14);//11.5
    public static final double zeroExtensionIn = 0.0;

    
    // Limits To Angles
    public static final double minAngleRad = Units.degreesToRadians(14); //33
    public static final double maxAngleRad = Units.degreesToRadians(115.0);
    
    // Limit to Extension
    public static final double minExtensionIn = 0;
    public static final double maxExtensionIn = 17.2;

    // Max Speeds
    public static final double maxPivotRateRadSec = Units.degreesToRadians(70);
    public static final double physicalMaxPivotRadSec = Units.degreesToRadians(142);
    
    public static final double maxPivotRatePercentSec = maxPivotRateRadSec/physicalMaxPivotRadSec;

    // Setpoints
    public static final double gearRatioExtension = 1.0/10.0;
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

    //Change ID and Inverted
    public static final int kIntakeID = 6;
    public static final TalonFXInvertType kIntakeInverted = TalonFXInvertType.Clockwise;
    
    public static final double kIntakeDeadZone = 0.05;
    public static final double kIntakeStallSpeed = -0;

    //Change this
    public static boolean leftPivotInverted = true;
}