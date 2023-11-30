package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    TalonFX frontMotor;
    TalonFX backMotor;
    int _frontMotorID;
    int _backMotorID;
    PIDController _velocityPID;

    public Shooter(int frontMotorID, int backMotorID, PIDController velocityPID) {
        _frontMotorID = frontMotorID;
        _backMotorID = backMotorID;
        frontMotor = new TalonFX(frontMotorID);
        backMotor = new TalonFX(backMotorID);
        _velocityPID = velocityPID;
    }

    public void shoot() {

    }
}
