package frc.robot.constants;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {

  private ArmConstants(){
    throw new IllegalStateException("Constants Class");
  }

    public enum ArmState {
        UNDEFINED(10000,10000),
        STOW(kMinAngleRad,kMinExtensionIn),

        LOWER_NODE_CONE(Units.degreesToRadians(30),0), //TBD
        LOWER_NODE_CUBE(Units.degreesToRadians(30),0), //TBD

        
        MID_NODE_CONE(Units.degreesToRadians(66.05),0), // Done
        MID_NODE_CUBE(Units.degreesToRadians(87.0),0), // TBD

        UPPER_NODE_CONE(Units.degreesToRadians(88.6),12.98), // EXT Calulated
        UPPER_NODE_CUBE(Units.degreesToRadians(100),17.1), //TBD

        SUBSTATION_CONE(Units.degreesToRadians(85.19),0), //TBD
        SUBSTATION_CUBE(Units.degreesToRadians(90),0), //TBD

        GROUND_PICKUP_CONE(Units.degreesToRadians(47),16.7), //Calculated
        //125 cm away from center
        GROUND_PICKUP_CUBE(Units.degreesToRadians(40),9.5); //Not popssible
        public double pivotAngleRad;
        public double extentionDistIn;
        ArmState(double piv, double ext){
          pivotAngleRad = piv;
          extentionDistIn = ext;
        }
      }
        
    public static final double kArmStatePivDeadzoneRad = Units.degreesToRadians(3);
    public static final double kArmStateExtDeadzoneIn = 2; 

    // Common Configurables
    public static final boolean kRateLimitArm = false;
    public static final boolean kUseFeedForward = true;
    public static final double kG = 0.03;

    //channel on dio port
    public static final int kDIOPortPiv = 0;
    public static final double kPivotAbsEncToRotation = 1.0/3.7142857;
    public static final double kExtensionRotationToInches =  18.5/7.01;



    // Offset For Pivot
    public static final double kPivotInitOffsetRot = .032;
    
    //Initial Value
    public static final double kZeroExtensionIn = 0.0;

    
    // Limits To Angles
    public static final double kMinAngleRad = Units.degreesToRadians(14); //33
    public static final double kMaxAngleRad = Units.degreesToRadians(115.0);
    
    // Limit to Extension
    public static final double kMinExtensionIn = 0;
    public static final double kMaxExtensionIn = 17.2;

    // Max Speeds
    public static final double kMaxPivotRateRadSec = Units.degreesToRadians(70);
    public static final double kPhysicalMaxPivotRadSec = Units.degreesToRadians(142);
    public static final double kMaxPivotRatePercentSec = kMaxPivotRateRadSec/kPhysicalMaxPivotRadSec;

    // Setpoints
    public static final double kGearRatioExtension = 1.0/10.0;
    public static final double kPivotPosInMetersY = Units.inchesToMeters(45.75);

    //IDs
    public static final int kRightArmPivotID = 16;
    public static final int kleftArmPivotID = 15;
    public static final int kTelescopicArmSparkID = 10;
    public static final int kArmPivotAbsEncoderPort = 17;
    
    //Gear Ratios
    public static final double kFalconToFinalGear = 1.0/240;
    public static final double kEncoderResolution = 1.0/2048;

    //Inversions
    public static final boolean kLeftPivotInverted = true;
    public static final boolean kRightPivotInverted = false;

}