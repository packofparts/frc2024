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
    private int _leftMotorID;
    private int _motorID;
    private int _encoderID;
    private double PIDOutput;

    private static PIDController pid;
    public CANSparkMax _motor;
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
    public void move(double inches) {
        PIDOutput = pid.calculate(calculateInches(_encoder.getAbsolutePosition()), inches);
        _motor.set(PIDOutput);
        if (calculateInches(_encoder.getAbsolutePosition()) >= inches) {
            _motor.stopMotor();
            _motor.stopMotor();
        }
    }

    public static double calculateInches(double degrees) {
        double inches = (WcConstants.WcCircumference / 360) * degrees;
        return inches;
    }
}


