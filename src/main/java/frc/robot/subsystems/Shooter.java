package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    TalonFX frontMotor;
    TalonFX backMotor;
    int _frontMotorID;
    int _backMotorID;
    PIDController _velocityPIDFront;
    PIDController _velocityPIDBack;


    public Shooter(int frontMotorID, int backMotorID, PIDController velocityPIDFront,
            PIDController velocityPIDBack) {
        _frontMotorID = frontMotorID;
        _backMotorID = backMotorID;
        frontMotor = new TalonFX(frontMotorID);
        backMotor = new TalonFX(backMotorID);
        _velocityPIDFront = velocityPIDFront;
        _velocityPIDBack = velocityPIDBack;
    }

    public void shoot(double velocitySP) {
        double PIDOutputFront = _velocityPIDFront.calculate(velocitySP);
        frontMotor.set(PIDOutputFront);
        double PIDOutputBack = _velocityPIDBack.calculate(velocitySP);
        backMotor.set(PIDOutputBack);
    }
}
