package frc.robot.constants;

public class SwerveConstants {
  private SwerveConstants(){
    throw new IllegalStateException("Constants Class");
  }

  public static final double kTransGearRatio = 1 / 6.75;
  public static final boolean kDebugMode = true;

  // Gear Conversions
  public static final double kWheelDiamMeters = .1016;
  public static final double kWheelCircumference = Math.PI * kWheelDiamMeters;

  // Conversion Factors
  public static final double kTransRPMtoMPS = (kTransGearRatio * kWheelCircumference) / 60;

  // Track dimensions in meters
  public static final double kTrackWidthMeters = .54;
  public static final double kTrackLengthMeters = .54;


  // Physical Max
  public static final double kPhysicalMaxSpeedMPS = 4.3;

  public static final double kTeleMaxSpeedMPS = 4.3;
  public static final double kTeleMaxRotSpeedRadPerSeconds = 2*Math.PI;

  public static final double kTeleMaxAccMPS = 5.0;
  public static final double kTeleMaxRotAccRadPerSeconds = 4*Math.PI;

  public static final boolean kPIDTuneMode = false;

}
