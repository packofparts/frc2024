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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    // Parameters
    private int _rotID;
    private int _transID;
    private int _rotEncoderID;
    private double _rotEncoderOffset;
    private boolean _rotInverse;
    private boolean _transInverse;
    private PIDController _rotPID;

    // Hardware
    // Motor Controllers
    private CANSparkMax _rotMotor;
    private CANSparkMax _transMotor;
    // Encoders
    private CANCoder _rotEncoder;
    private RelativeEncoder _transEncoder;


    public double PIDOutput = 0.0;
    public SwerveModule(int rotID, int transID, int rotEncoderID, double rotEncoderOffset,
            boolean rotInverse, boolean transInverse, PIDController rotPID) {
        // Setting Parameters
        _rotID = rotID;
        _transID = transID;
        _rotEncoderID = rotEncoderID;
        _rotEncoderOffset = rotEncoderOffset;
        _rotInverse = rotInverse;
        _transInverse = transInverse;

        // ----Setting Hardware
        // Motor Controllers
        _rotMotor = new CANSparkMax(_rotID, MotorType.kBrushless);
        _transMotor = new CANSparkMax(_transID, MotorType.kBrushless);
        

        // Encoders
        _rotEncoder = new CANCoder(_rotEncoderID);
        _transEncoder = _transMotor.getEncoder();

        // Sets measurement to radians
        // CANCoderConfiguration configuration = getCANCoderConfig(rotEncoderOffset, rotInverse);
        // _rotEncoder.configAllSettings(configuration);


        // ----Setting PID
        _rotPID = rotPID;

        // ----Setting PID Parameters
        _rotPID.enableContinuousInput(0, 2*Math.PI);



        // ----Setting Inversion
        _rotMotor.setInverted(_rotInverse);
        _transMotor.setInverted(_transInverse);
    }

    private CANCoderConfiguration getCANCoderConfig(double offset, boolean inverse){
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = 2*Math.PI / 4096;
        config.unitString = "rot";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        
        config.magnetOffsetDegrees = offset*360-180;
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = inverse;

        return config;
    }
    
    // ------------------- State Settings

    /**
     * 
     * @return a swerve module state object describing the current speed of the translation and
     *         rotation motors
     * @see SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getTransVelocity(), Rotation2d.fromRadians(getRotPosition()));
    }

    /**
     * Sets the motor speeds passed into constructor
     * 
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Stops returning to original rotation
        // if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
        //     stop();
        //     return;
        // }

        // No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // PID Controller for both translation and rotation
        // _transMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeedMPS);
        
        PIDOutput = _rotPID.calculate(getRotPosition(), desiredState.angle.getRadians());

        _rotMotor.set(PIDOutput);

//        _rotMotor.set(0.1);

        

    }

    /**
     * 
     * @return the total distance traveled by the module (Meters) and Rotation value (Rad) in the
     *         form of a SwerveModulePostion object
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getModulePos() {
        return new SwerveModulePosition(getTransPosition(),
                Rotation2d.fromRadians(getRotPosition()));
    }

    // -------------------- Get Raw Values

    /**
     * 
     * @return Returns number rotations of translation motor BEFORE GEAR RATIO
     */
    public double getTransPositionRaw() {
        return _transEncoder.getPosition();
    }

    /**
     * 
     * @return Returns rotation position in radians
     */
    public double getRotPositionRaw() {
        return _rotEncoder.getAbsolutePosition();
    }

    /**
     * 
     * @return Returns velocity of translation motor BEFORE GEAR RATIO
     */
    public double getTransVelocityRaw() {
        return _transEncoder.getVelocity();
    }

    // -------------------- Applying Conversions/Rollover

    /**
     * 
     * @return Returns translation motor AFTER GEAR RATIO and Meters
     */
    public double getTransPosition() {
        return getTransPositionRaw() * SwerveConstants.kTransGearRatio
                * SwerveConstants.kDriveGearToMeters;
    }

    /**
     * 
     * @return Returns rotation in RADIANS of rotation motor AFTER GEAR RATIO
     */
    public double getRotPosition() {
        return getRotPositionRaw();
    }

    /**
     * 
     * @return Returns velocity of translation motor with conversion
     */
    public double getTransVelocity() {
        return getTransVelocityRaw() * SwerveConstants.kTransRPMtoMPS;
    }


    /**
     * Reset ONLY the translation encoder
     */
    public void resetEncoders() {
        _transEncoder.setPosition(0);
    }

    /**
     * Stops the both motors
     */
    public void stop() {
        _transMotor.set(0);
        _rotMotor.set(0);
    }

    /**
     * 
     * @return steering PID controller.
     */
    public PIDController getPIDController() {
        return this._rotPID;
    }

    /**
     * Sets the mode of the translation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeTrans(IdleMode mode) {
        _transMotor.setIdleMode(mode);
    }

    /**
     * Permanently burns settings into the sparks
     * 
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks() {
        _rotMotor.burnFlash();
        _transMotor.burnFlash();
    }

    /**
     * Sets the mode of the rotation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeRot(IdleMode mode) {
        _rotMotor.setIdleMode(mode);
    }
}
