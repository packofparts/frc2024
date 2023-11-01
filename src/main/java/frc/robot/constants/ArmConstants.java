package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {

  private ArmConstants() {
    throw new IllegalStateException("Constants Class");
  }

  public enum ArmState {
    UNDEFINED(10000, 10000), STOW(MIN_PIV_ANGLE_RAD, MIN_EXT_LEN_IN),

    LOWER_NODE_CONE(Units.degreesToRadians(30), 0), // TBD
    LOWER_NODE_CUBE(Units.degreesToRadians(30), 0), // TBD

    MID_NODE_CONE(Units.degreesToRadians(66.05), 0), // Done
    MID_NODE_CUBE(Units.degreesToRadians(87.0), 0), // TBD

    UPPER_NODE_CONE(Units.degreesToRadians(88.6), 12.98), // EXT Calulated
    UPPER_NODE_CUBE(Units.degreesToRadians(100), 17.1), // TBD

    SUBSTATION_CONE(Units.degreesToRadians(85.19), 0), // TBD
    SUBSTATION_CUBE(Units.degreesToRadians(90), 0), // TBD

    GROUND_PICKUP_CONE(Units.degreesToRadians(47), 16.7), // Calculated
    // 125 cm away from center
    GROUND_PICKUP_CUBE(Units.degreesToRadians(40), 9.5); // Not popssible

    public final double pivotAngleRad;
    public final double extentionDistIn;

    ArmState(double piv, double ext) {
      pivotAngleRad = piv;
      extentionDistIn = ext;
    }
  }

  // Common Configurables
  public static final boolean RATE_LIMIT_ARM = false;
  public static final boolean ENABLE_FEEDFORWARD = true;
  public static final double KG = 0.03;

  // channel on dio port
  public static final int DIOPORTPIV = 0;
  public static final int CONNECTION_THRESH_HZ = 975;


  public static final double PIVOT_ABS_ENC_TO_ROTATION = 1.0 / 3.7142857;
  public static final double EXTENSION_ROTATION_TO_INCHES = 18.5 / 7.01;


  // Offset For Pivot
  public static final double PIV_INIT_OFFSET_ROT = .032;

  // Initial Value
  public static final double ZERO_EXTENSION_IN = 0.0;


  // Limits To Angles
  public static final double MIN_PIV_ANGLE_RAD = Units.degreesToRadians(14); // 33
  public static final double MAX_PIV_ANGLE_RAD = Units.degreesToRadians(115.0);

  // Limit to Extension
  public static final double MIN_EXT_LEN_IN = 0;
  public static final double MAX_EXT_LEN_IN = 17.2;

  // Max Speeds
  public static final double MAX_PIV_RATE_RAD_SEC = Units.degreesToRadians(70);
  public static final double PHYSICAL_MAX_PIV_RAD_SEC = Units.degreesToRadians(142);
  public static final double MAX_PIV_RATE_PERCENT_SEC =
      MAX_PIV_RATE_RAD_SEC / PHYSICAL_MAX_PIV_RAD_SEC;

  // Setpoints
  public static final double GEAR_RATIO_EXTENSION_IN = 8.333 / 8.113;
  public static final double Y_PIVOT_POS_METERS = Units.inchesToMeters(45.75);

  // IDs
  public static final int RIGHT_PIV_MOTOR_ID = 16;
  public static final int LEFT_PIV_MOTOR_ID = 15;
  public static final int EXT_SPARK_ID = 10;
  public static final int PIV_ABSENC_PORT = 17;

  // Gear Ratios
  public static final double PIV_MOTOR_TO_GEAR_ROT = 1.0 / 240;
  public static final double ENCODER_RES = 1.0 / 2048;

  // Inversions
  public static final boolean LEFT_PIV_MOTOR_INVERTED = true;
  public static final boolean RIGHT_PIV_MOTOR_INVERTED = false;

  // Tolerances
  public static final double RESTING_PIV_TOLERANCE_RAD = Units.degreesToRadians(0);
  public static final double RESTING_EXT_TOLERANCE_IN = 0.2;

  public static final double ACTIVE_PIV_TOLERANCE_RAD = Units.degreesToRadians(4);
  public static final double ACTIVE_EXT_TOLERANCE_IN = 0.5;

  public static final double EXT_FRICTION_COEFF = .005;
  public static final double EXT_FRICTION_ACTIVATION_THRESH = .14;

  public static final double EXT_MAX_SPEED_CLAMP_PERCENT = 0.8;
  public static final double PIV_MAX_SPEED_CLAMP_PERCENT = 0.8;

  public static final double EXT_MAX_PID_CONTRIBUTION_PERCENT = 0.5;
  public static final double PIV_MAX_PID_CONTRIBUTION_PERCENT = 0.55;

  public static final double PIV_MAX_KG_CONTRIBUTION_PERCENT = 0.3;
}
