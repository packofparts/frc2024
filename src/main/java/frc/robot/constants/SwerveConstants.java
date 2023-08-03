package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class SwerveConstants {

  // Ratio for Mk2 Swerve Modules
  // Found https://www.chiefdelphi.com/t/2910-mk2-swerve-module-release/335077
  // and https://www.swervedrivespecialties.com/products/mk2-module-kit?variant=31141088821361
  public static final double kRotGearRatio = 1 / 18;
  public static final double kTransGearRatio = 1 / 8.33;

  // Gear Conversions
  public static final double kDriveGearToMeters = (2 * Math.PI) * 2;

  // Conversion Factors
  public static final double kTransRPMtoMPS = (kTransGearRatio * kDriveGearToMeters) / 60;

  // Track dimensions in meters
  public static final double kTrackWidthMeters = .54;
  public static final double kTrackLengthMeters = .54;


  // Physical Max
  public static final double kPhysicalMaxSpeedMPS = 4.0;

  public static final double kTeleMaxSpeedMPS = 4.0;
  public static final double kAutoMaxSpeedMPS = 3.0;

  // Encoder Offsets
  public static double kfrontRightRotEncoderOffset = 0.228+.5;
  public static double kfrontLeftRotEncoderOffset = 0.966-.5;
  public static double kbackRightRotEncoderOffset = 0.045+.5;
  public static double kbackLeftRotEncoderOffset = 0.28+.5;


}
