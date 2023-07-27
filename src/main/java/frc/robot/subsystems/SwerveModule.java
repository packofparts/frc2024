package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class SwerveModule {
    // Parameters 
    private int _rotID;
    private int _transID;
    private int _rotEncoderID;
    private int _transEncoderID;
    private double _rotEncoderOffset;
    private boolean _rotInverse;
    private boolean _transInverse;
    private PIDController _rotPID;
    private PIDController _transPID;

    // Hardware
    // Motor Controllers
    private CANSparkMax _rotMotor;
    private CANSparkMax _transMotor;
    // Encoders
    private AnalogEncoder _rotEncoder;
    private RelativeEncoder _transEncoder;

    public SwerveModule(
        int rotID,
        int transID,
        int rotEncoderID,
        int transEncoderID,
        double rotEncoderOffset,
        boolean rotInverse,
        boolean transInverse,
        PIDController rotPID,
        PIDController transPID
    ) {
        // Setting Parameters
        _rotID = rotID;
        _transID = transID;
        _rotEncoderID = rotEncoderID;
        _transEncoderID = transEncoderID;
        _rotEncoderOffset = rotEncoderOffset;
        _rotInverse = rotInverse;
        _transInverse = transInverse;

        // ----Setting Hardware
        // Motor Controllers
        _rotMotor = new CANSparkMax(_rotID, MotorType.kBrushless);
        _transMotor = new CANSparkMax(_transID, MotorType.kBrushless);

        // Encoders
        _rotEncoder = new AnalogEncoder(_rotEncoderID);
        _transEncoder = _transMotor.getEncoder();

        
        _rotEncoder.setPositionOffset(_rotEncoderOffset);


        // ----Setting PID
        _rotPID = rotPID;
        _transPID = transPID;

        // ----Setting PID Parameters
        _rotPID.enableContinuousInput(-Math.PI,Math.PI);



        // ----Setting Inversion
        _rotMotor.setInverted(_rotInverse);
        _transMotor.setInverted(_transInverse);
    }

    // ------------------- State Settings

    /**
     * 
     * @return a swerve module state object describing the current speed of the translation and rotation motors
     * @see SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getTransVelocity(), new Rotation2d(getRotPosition()));  
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
                _transMotor.set(desiredState.speedMetersPerSecond/SwerveConstants.kPhysicalMaxSpeedMPS);
                break;
            case TELEOP:
                _transMotor.set(desiredState.speedMetersPerSecond/SwerveConstants.kPhysicalMaxSpeedMPS);
                break;
        }

        
        _rotMotor.set(
            _rotPID.calculate(getRotPosition() * 2 * Math.PI,
                desiredState.angle.getRotations())
        );
  
    }

    /**
     * 
     * @return the total distance traveled by the module (Meters) and Rotation value (Rad) in the form of a SwerveModulePostion object
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getModulePos(){
            return new SwerveModulePosition(getTransPosition(), new Rotation2d.fromRotations(getRotPosition()));
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
     * @return Returns number rotations of rotation motor BEFORE GEAR RATIO
     */
    public double getRotPositionRaw() {
        return _rotEncoder.getAbsolutePosition() - _rotEncoder.getPositionOffset();
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
        return  getTransPositionRaw() * SwerveConstants.transGearRatio * SwerveConstants.driveGearToMeters;
    }

    /**
     * 
     * @return Returns number rotations of rotation motor AFTER GEAR RATIO 
     */
    public double getRotPosition() {
        return -getRotPositionRaw();
    }

    /**
     * 
     * @return Returns velocity of translation motor with conversion
     */
    public double getTransVelocity() {
        return getTransVelocityRaw() * SwerveConstants.transRPMtoMPS;
    }


    /**
     * 
     * @param offset offset of rotation encoder in rotations from 0 to 1
     * @return Set the offset of the rotation encoder in rotations
     */

    public void setRotationOffset(double offset){
        _rotEncoder.setPositionOffset(offset);
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
    public PIDController getPIDController(){
        return this._rotPID;
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
}
