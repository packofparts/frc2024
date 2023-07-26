package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class SwerveConstants {
    

    // Ratio for Mk2 Swerve Modules
    // Found https://www.chiefdelphi.com/t/2910-mk2-swerve-module-release/335077
    // and https://www.swervedrivespecialties.com/products/mk2-module-kit?variant=31141088821361 
    public static final double rotGearRatio = 1/18;
    public static final double transGearRatio = 1/8.33;

    // Gear Conversions
    public static final double driveGearToMeters = (2 * Math.PI) * 2;

    // Conversion Factors
    public static final double transRPMtoMPS = (transGearRatio * driveGearToMeters) / 60;


    // Physical Max
    public static final double kPhysicalMaxSpeedMPS = 4.0;
    
    

}
