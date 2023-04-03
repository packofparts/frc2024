package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.CompConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;
import frc.robot.Constants.MiscNonConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {    
    private int _motorTransID;
    private int _motorRotID;
    private int _analogEncoderID;
    public CANSparkMax _transMotor;
    private CANSparkMax _rotMotor;
    private RelativeEncoder _transEncoder;
    private RelativeEncoder _rotEncoder;
    public AnalogEncoder _analogEncoder;
    public PIDController _rotationPIDController;
    public PIDController _transController;
    
    private boolean _transInverted;
    private boolean _rotInverted;
    private boolean _isAbsEncoder;

    public SwerveModule(int motorTransID, int motorRotID, int universalEncoderID,
     boolean transInverted, boolean rotInverted, double universalEncoderOffsetinit,
     boolean universalEncoderInverted, boolean isAbsEncoder,PIDController pidController,PIDController transController){
        this._motorTransID = motorTransID;
        this._analogEncoderID = universalEncoderID;
        this._motorRotID = motorRotID;
        this._transInverted = transInverted;
        this._rotInverted = rotInverted;
        this._transController = transController;
        this._isAbsEncoder = isAbsEncoder;


        _transMotor = new CANSparkMax(this._motorTransID, MotorType.kBrushless);
        _rotMotor = new CANSparkMax(this._motorRotID, MotorType.kBrushless);

        if (isAbsEncoder){
            _analogEncoder = new AnalogEncoder(this._analogEncoderID);
            _analogEncoder.setPositionOffset(universalEncoderOffsetinit);
            SmartDashboard.putNumber("Offset", _analogEncoder.getPositionOffset());
        }

        _transMotor.setInverted(this._transInverted);
        _rotMotor.setInverted(this._rotInverted);

        _transEncoder = _transMotor.getEncoder();
        _rotEncoder = _rotMotor.getEncoder();

        _rotationPIDController = pidController;
        _rotationPIDController.enableContinuousInput(-Math.PI,Math.PI);       
        resetEncoders(); 

        if (CompConstants.reduceRelativeFrameRate) {
            _transEncoder.setMeasurementPeriod(CompConstants.reducedRelativeFrameRate);
        }

        // applySettings();
        // burnSparks();
    }

    /**
     * 
     * @return Returns rotations of translation motor BEFORE GEAR RATIO
     */
    public double getTransPosition() {
        return _transEncoder.getPosition();
    }

    public void setOffset(double offset){
        _analogEncoder.setPositionOffset(offset);
    }

    /**
     * @return Returns radians of rotation motor BEFORE GEAR RATIO, relative probably won't work at the moment
     */
    public double getRotPosition(){
        if(_isAbsEncoder){
            return -(
                _analogEncoder.getAbsolutePosition() - _analogEncoder.getPositionOffset()) * 2 * Math.PI;
        }else{
            return _rotEncoder.getPosition();
        } 
    }

    /**
     * @return returns RPM of Translation (GEAR RATIO ALREADY APPLIED THROUGH POSITIONCONVERION FACTOR)
     */
    public double getTransVelocity(){        
        return _transEncoder.getVelocity(); 
    }

    /**
     * 
     * @return returns RPM of Rotation BEFORE GEAR RATIO
     */
    public double getRotVelocity(){
        return _rotEncoder.getVelocity();
    }

    
    /**
     * Resets Relative encoder to Abs encoder position
     */
    public void resetEncoders(){

        if (_isAbsEncoder) {
            _rotEncoder.setPosition(-(
                _analogEncoder.getAbsolutePosition() - _analogEncoder.getPositionOffset()) * 18);
            _transEncoder.setPosition(0);
        } else {
            _transEncoder.setPosition(0);
            _rotEncoder.setPosition(0);
        }
    }

    /**
     * 
     * @return a swerve module state object describing the current speed of the translation and rotation motors
     * @see SwerveModuleState
     */
    public SwerveModuleState getState(){

        // Returns SwerveModuleState
        return new SwerveModuleState(getTransVelocity()*DriveConstants.RPMtoMPS,
            new Rotation2d(getRotPosition()));  
    }

    /**
     * Sets the motor speeds passed into constructor
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState, DriveMode dMode){
        
        // Stops returning to original rotation
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        //No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        //PID Controller for both translation and rotation
        switch(dMode){
            case AUTO:
                _transMotor.set(desiredState.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMPS);
                break;
            case TELEOP:
                _transMotor.set(desiredState.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMPS);
                break;
        }

        
        if(_isAbsEncoder){
            _rotMotor.set(_rotationPIDController.calculate(( -(
                _analogEncoder.getAbsolutePosition() - _analogEncoder.getPositionOffset()) ) * 2 * Math.PI,
            desiredState.angle.getRadians()));
        }else{
            _rotMotor.set(_rotationPIDController.calculate((_rotEncoder.getPosition())*DriveConstants.angleEncoderConversionFactortoRad,
            desiredState.angle.getRadians()));
        }
  
    }

    /**
     * This is only for PID tuning
     * @param setPoint PID setpoint
     */
    public void updatePositions(double setPoint){

        // FOR PID TUNING ONLY
        _rotationPIDController.setPID(MiscNonConstants.kP, MiscNonConstants.kI, MiscNonConstants.kD);
        _rotationPIDController.disableContinuousInput();
        double sp = _rotationPIDController.calculate((getRotPosition()-0.5)*2*Math.PI/18, setPoint);
        _rotMotor.set(sp);
    }


    /**
     * Call this in execute as it uses a PID controller
     */
    public void returnToOrigin(){
        // Sets wheel rot to original state
        _rotMotor.set(_rotationPIDController.calculate(((getRotPosition()%18)*2*Math.PI/18), 0));
        _rotationPIDController.setTolerance(0);
    }

    /**
     * 
     * @return the total distance traveled by the module (Meters) and Rotation value (Rad) in the form of a SwerveModulePostion object
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getModulePos(){
        if(_isAbsEncoder){
            return new SwerveModulePosition(_transEncoder.getPosition()*DriveConstants.kDriveEncoderRot2Meter,
            new Rotation2d(getRotPosition()));
        }else{
            return new SwerveModulePosition(_transEncoder.getPosition()*DriveConstants.kDriveEncoderRot2Meter,
            new Rotation2d(getRotPosition()*DriveConstants.angleEncoderConversionFactortoRad));
        }
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
    public PIDController getPIDController(){
        return this._rotationPIDController;
    }

    /**
     * Sets the mode of the translation motor
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeTrans(IdleMode mode){
        _transMotor.setIdleMode(mode);
    }

    /**
     * Permanently burns settings into the sparks
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks(){
        _rotMotor.burnFlash();
        _transMotor.burnFlash();
    }

    /**
     * Sets the mode of the rotation motor
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeRot(IdleMode mode){
        _rotMotor.setIdleMode(mode);
    }

    public void applySettings(){
        _transEncoder.setPositionConversionFactor(1.0/8.31); //set to a constant
    }
}