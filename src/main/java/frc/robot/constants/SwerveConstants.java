package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class SwerveConstants {

  // Ratio for Mk2 Swerve Modules
  // Found https://www.chiefdelphi.com/t/2910-mk2-swerve-module-release/335077
  // and https://www.swervedrivespecialties.com/products/mk2-module-kit?variant=31141088821361
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

  //mk2 Encoder Offsets remove
  public static double kfrontRightRotEncoderOffset = 0.228+.5-0.79;
  public static double kfrontLeftRotEncoderOffset = 0.966-.5;
  public static double kbackRightRotEncoderOffset = 0.721;
  public static double kbackLeftRotEncoderOffset = 0.699;

  public static boolean kPIDTuneMode = false;

}
