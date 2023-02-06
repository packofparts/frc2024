package frc.robot;

import edu.wpi.first.math.util.Units;

public class Util {
    
    public static double clamp(double toClamp, double min, double max){
        toClamp = Math.min(toClamp, max);
        toClamp = Math.max(toClamp, min);
        
        return toClamp;
    }

    public static double degreesToOffsettedRadians(double degrees, double absEncoderOffset){
        return Units.degreesToRadians(degrees) - Units.rotationsToRadians(absEncoderOffset);
    }
}