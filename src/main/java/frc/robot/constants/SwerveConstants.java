package frc.robot.constants;

public class SwerveConstants {
  private SwerveConstants() {
    throw new IllegalStateException("Constants Class");
  }

  // Gear Conversions
  public static final double TRANS_GEAR_RATIO_ROT = 1 / 6.75;
  public static final double REL_ENC_GEAR_RATIO_ROT = 1 / 12.8;
  public static final double ABS_ENC_GEAR_RATIO_ROT = 1;

  // Conversion Factors
  public static final double WHEEL_DIAMETER_METERS = .1016;
  public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
  public static final double TRANS_RPM_TO_MPS =
      (TRANS_GEAR_RATIO_ROT * WHEEL_CIRCUMFERENCE_METERS) / 60;

  // Track dimensions in meters
  public static final double TRACK_WIDTH_METERS = .54;
  public static final double TRACK_LENGTH_METERS = .54;



  // Physical Max
  public static final double PHYSICAL_MAX_SPEED_MPS = 4.3;

  public static final double TELE_MAX_SPEED_MPS = 4.3;
  public static final double TELE_MAX_ROT_SPEED_RAD_SEC = 2 * Math.PI;

  public static final double TELE_MAX_ACC_MPS2 = 5.0;
  public static final double TELE_MAX_ROT_ACC_RAD_SEC2 = 4 * Math.PI;


  public static final double GYRO_ANGLE_OFFSET = 20; // 7.31; // Degrees per Second
}
