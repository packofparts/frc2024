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
    private final int mRotID;
    private final int mTransID;
    private final int mRotEncoderID;
    private final boolean mRotInverse;
    private final boolean mTransInverse;
    private final PIDController mRotPID;

    // Hardware
    // Motor Controllers
    private final CANSparkMax mRotMotor;
    private final CANSparkMax mTransMotor;
    // Encoders
    private final CANCoder mRotEncoder;
    private final RelativeEncoder mTransEncoder;
    private final RelativeEncoder mRotRelativeEncoder;

    // Public Debugging Values
    private double mPIDOutput = 0.0;
    private double mDesiredRadians = 0.0;

    public SwerveModule(int rotID, int transID, int rotEncoderID, boolean rotInverse,
            boolean transInverse, PIDController rotPID) {
        // Setting Parameters
        mRotID = rotID;
        mTransID = transID;
        mRotEncoderID = rotEncoderID;
        mRotInverse = rotInverse;
        mTransInverse = transInverse;

        // ----Setting Hardware
        // Motor Controllers
        mRotMotor = new CANSparkMax(mRotID, MotorType.kBrushless);
        mTransMotor = new CANSparkMax(mTransID, MotorType.kBrushless);


        // Encoders
        mRotEncoder = new CANCoder(mRotEncoderID);
        mTransEncoder = mTransMotor.getEncoder();
        mRotRelativeEncoder = mRotMotor.getEncoder();
        mRotRelativeEncoder.setPosition(0);
        // Sets measurement to radians

        // ----Setting PID
        mRotPID = rotPID;

        // ----Setting PID Parameters
        mRotPID.enableContinuousInput(-Math.PI, Math.PI);

        // ----Setting Inversion
        mRotMotor.setInverted(mRotInverse);
        mTransMotor.setInverted(mTransInverse);

        mTransMotor.setIdleMode(IdleMode.kBrake);
        mRotMotor.setIdleMode(IdleMode.kBrake);

        mTransEncoder.setPosition(0);
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

    public double getAppliedOutput() {
        return mRotMotor.getAppliedOutput();
    }

    public void setTransMotorRaw(double speed) {
        mTransMotor.set(speed);
    }

    public void setRotMotorRaw(double speed) {
        mRotMotor.set(speed);

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
        mTransMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.PHYSICAL_MAX_SPEED_MPS);
        mDesiredRadians = desiredState.angle.getRadians();
        mPIDOutput = mRotPID.calculate(getRotPosition(), desiredState.angle.getRadians());

        mRotMotor.set(mPIDOutput);

    }

    public void setPID(double degrees) {
        mPIDOutput = mRotPID.calculate(getRotPosition(), Math.toRadians(degrees));
        mRotMotor.set(mPIDOutput);
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
    private double getTransPositionRaw() {
        return mTransEncoder.getPosition();
    }

    /**
     * 
     * @return Returns rotation position in radians
     */
    private double getRotPositionRaw() {
        return mRotEncoder.getAbsolutePosition();
    }

    /**
     * 
     * @return Returns velocity of translation motor BEFORE GEAR RATIO
     */
    private double getTransVelocityRaw() {
        return mTransEncoder.getVelocity();
    }

    // -------------------- Applying Conversions/Rollover

    /**
     * 
     * @return Returns translation motor AFTER GEAR RATIO and Meters
     */
    public double getTransPosition() {
        return getTransPositionRaw() * SwerveConstants.TRANS_GEAR_RATIO_ROT
                * SwerveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    /**
     * 
     * @return Returns rotation in RADIANS of rotation motor AFTER GEAR RATIO
     */
    public double getRotPosition() {
        return getRotPositionRaw() * SwerveConstants.ABS_ENC_GEAR_RATIO_ROT;
    }

    public double getRotRelativePosition() {
        return mRotRelativeEncoder.getPosition() * SwerveConstants.REL_ENC_GEAR_RATIO_ROT;
    }

    /**
     * 
     * @return Returns velocity of translation motor with conversion
     */
    public double getTransVelocity() {
        return getTransVelocityRaw() * SwerveConstants.TRANS_RPM_TO_MPS;
    }


    /**
     * Reset ONLY the translation encoder
     */
    public void resetEncoders() {
        mTransEncoder.setPosition(0);
    }

    /**
     * Stops the both motors
     */
    public void stop() {
        mTransMotor.set(0);
        mRotMotor.set(0);
    }

    /**
     * 
     * @return steering PID controller.
     */
    public PIDController getPIDController() {
        return mRotPID;
    }

    /**
     * Sets the mode of the translation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeTrans(IdleMode mode) {
        mTransMotor.setIdleMode(mode);
    }

    /**
     * Permanently burnCs settings into the sparks
     * 
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks() {
        mRotMotor.burnFlash();
        mTransMotor.burnFlash();
    }

    /**
     * Sets the mode of the rotation motor
     * 
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeRot(IdleMode mode) {
        mRotMotor.setIdleMode(mode);
    }

    /**
     * Retrives the rotation PID output provided to the motors after desaturation and optimization
     */
    public double getPIDOutputRot() {
        return mPIDOutput;
    }

    /**
     * Retrives the desired radian setpoint of rotation of the motors after desaturation and
     * optimization.
     */
    public double getDesiredRadiansRot() {
        return mDesiredRadians;
    }
}
