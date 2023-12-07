package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;
import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.*;
import frc.robot.constants.WcConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class wcModule {
    // Parameters
    private int _motorID;
    private int _encoderID;

    private static PIDController pid;
    private CANSparkMax _motor;
    private static CANCoder _encoder;

    public wcModule(int motorID, int encoderID) {
        _motorID = motorID;
        _encoderID = encoderID;

        // CHECK MOTOR (IF BRUSHLESS)
        _motor = new CANSparkMax(_motorID, MotorType.kBrushless);

        // Encoders
        _encoder = new CANCoder(_encoderID);

    }

    // method that moves the robot forward a certain distance
    public boolean move(double inches) {
        // Stop moving once robot is within 0.1 in. of target
        if (calculateInches(_encoder.getPosition()) + 0.1 >= inches) {
            _motor.stopMotor();
            return true;
        }

        double PIDOutput = pid.calculate(calculateInches(_encoder.getPosition()), inches);
        _motor.set(PIDOutput);
        return false;
    }

    // Not used for now
    public void maintainVelocity(double velocity) {
        if ((getVelocityInches() < velocity) && (velocity != 0)) {
            double PIDOutput = pid.calculate(getVelocityInches(), velocity);
            _motor.set(Math.min(0.2, PIDOutput)); // Cap the motor output at 0.2 to avoid
                                                  // involuntary manslaughter
        } else {
            _motor.set(0);
        }
    }

    public static double calculateInches(double degrees) {
        double inches = (WcConstants.WcCircumference / 360) * degrees;
        return inches;
    }

    public double getVelocityInches() {
        return _encoder.getVelocity() / 360 * WcConstants.WcCircumference;
    }

    public void invert() {
        _motor.setInverted(!(_motor.getInverted()));
    }

    public void reset() {
        _encoder.setPosition(0);
    }
}


