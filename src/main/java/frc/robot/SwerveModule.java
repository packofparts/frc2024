package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    // Parameters
    private int rotID;
    private int transID;
    private int rotEncoderID;
    private boolean rotInverse;
    private boolean transInverse;
    private PIDController rotPID;

    // Hardware
    // Motor Controllers
    private CANSparkMax rotMotor;
    private CANSparkMax transMotor;
    // Encoders
    private CANCoder rotEncoder;
    private RelativeEncoder transEncoder;
    private RelativeEncoder rotRelativeEncoder;

    //Public Debugging Values
    public double PIDOutput = 0.0;
    public double desiredRadians = 0.0;

    public SwerveModule(int tmpRotID, int tmpTransID, int tmpRotEncoderID,
            boolean tmpRotInverse, boolean tmpTransInverse, PIDController tmpRotPID) {
        // Setting Parameters
        rotID = tmpRotID;
        transID = tmpTransID;
        rotEncoderID = tmpRotEncoderID;
        rotInverse = tmpRotInverse;
        transInverse = tmpTransInverse;

        // ----Setting Hardware
        // Motor Controllers
        rotMotor = new CANSparkMax(rotID, MotorType.kBrushless);
        transMotor = new CANSparkMax(transID, MotorType.kBrushless);
        

        // Encoders
        rotEncoder = new CANCoder(rotEncoderID);
        transEncoder = transMotor.getEncoder();
        rotRelativeEncoder = rotMotor.getEncoder();
        rotRelativeEncoder.setPosition(0);
        // Sets measurement to radians

        // ----Setting PID
        rotPID = tmpRotPID;

        // ----Setting PID Parameters
        tmpRotPID.enableContinuousInput(-Math.PI, Math.PI);

        // ----Setting Inversion
        rotMotor.setInverted(rotInverse);
        transMotor.setInverted(transInverse);

        transMotor.setIdleMode(IdleMode.kBrake);
        rotMotor.setIdleMode(IdleMode.kBrake);

        transEncoder.setPosition(0);
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

    public double getAppliedOutput(){
        return rotMotor.getAppliedOutput();
    }

    public void setTransMotorRaw(double speed){
        transMotor.set(speed);
    }

    public void setRotMotorRaw(double speed){
        rotMotor.set(speed);

    }

    /**
     * Sets the motor speeds passed into constructor
     * 
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Stops returning to original rotation
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // PID Controller for both translation and rotation
        transMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeedMPS);
        desiredRadians = desiredState.angle.getRadians();
        PIDOutput = rotPID.calculate(getRotPosition(), desiredState.angle.getRadians());

        rotMotor.set(PIDOutput);


    }

    public void setPID(double degrees){
        PIDOutput = rotPID.calculate(getRotPosition(),Math.toRadians(degrees));
        rotMotor.set(PIDOutput);

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
        return transEncoder.getPosition();
    }

    /**
     * 
     * @return Returns rotation position in radians
     */
    public double getRotPositionRaw() {
        return rotEncoder.getAbsolutePosition();
    }

    /**
     * 
     * @return Returns velocity of translation motor BEFORE GEAR RATIO
     */
    public double getTransVelocityRaw() {
        return transEncoder.getVelocity();
    }

    // -------------------- Applying Conversions/Rollover

    /**
     * 
     * @return Returns translation motor AFTER GEAR RATIO and Meters
     */
    public double getTransPosition() {
        return getTransPositionRaw() * SwerveConstants.kTransGearRatio
                * SwerveConstants.kWheelCircumference;
    }

    /**
     * 
     * @return Returns rotation in RADIANS of rotation motor AFTER GEAR RATIO
     */
    public double getRotPosition() {
        return getRotPositionRaw();
    }

    public double getRotRelativePosition(){
        return rotRelativeEncoder.getPosition()/12.8;
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
        transEncoder.setPosition(0);
    }

    /**
     * Stops the both motors
     */
    public void stop() {
        transMotor.set(0);
        rotMotor.set(0);
    }

    /**
     * 
     * @return steering PID controller.
     */
    public PIDController getPIDController() {
        return this.rotPID;
    }

    /**
     * Sets the mode of the translation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeTrans(IdleMode mode) {
        transMotor.setIdleMode(mode);
    }

    /**
     * Permanently burnCs settings into the sparks
     * 
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks() {
        rotMotor.burnFlash();
        transMotor.burnFlash();
    }

    /**
     * Sets the mode of the rotation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeRot(IdleMode mode) {
        rotMotor.setIdleMode(mode);
    }
}
