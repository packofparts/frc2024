package frc.robot;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {

    private int m_MotorTransID;
    private int m_MotorRotID;
    private int m_UniversalEncoderID;
    public CANSparkMax transMotor;
    private CANSparkMax rotMotor;
    private RelativeEncoder transEncoder;
    private RelativeEncoder rotEncoder;
    public AnalogEncoder universalEncoder;
    public SparkMaxPIDController rotPID;
    public PIDController rotationPIDController;
    public PIDController transController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    private Boolean isAbsoluteEncoder;
    private Boolean m_transInverted;
    private Boolean m_rotInverted;    

    public SwerveModule(int motorTransID, int motorRotID, int universalEncoderID,
     Boolean transInverted, Boolean rotInverted, double universalEncoderOffsetinit,
     Boolean universalEncoderInverted, boolean isAbsEncoder,PIDController pidController,PIDController transController){
        this.isAbsoluteEncoder=isAbsEncoder;
        this.m_MotorTransID = motorTransID;
        this.m_UniversalEncoderID = universalEncoderID;
        this.m_MotorRotID = motorRotID;
        this.m_transInverted = transInverted;
        this.m_rotInverted = rotInverted;

        transMotor = new CANSparkMax(this.m_MotorTransID, MotorType.kBrushless);
        
        rotMotor = new CANSparkMax(this.m_MotorRotID, MotorType.kBrushless);

        if (isAbsEncoder){
            universalEncoder = new AnalogEncoder(this.m_UniversalEncoderID);
            universalEncoder.setPositionOffset(universalEncoderOffsetinit);
            SmartDashboard.putNumber("Offset", universalEncoder.getPositionOffset());
        }


        transMotor.setInverted(this.m_transInverted);
        rotMotor.setInverted(this.m_rotInverted);


        transEncoder = transMotor.getEncoder();
        rotEncoder = rotMotor.getEncoder();
        rotationPIDController = pidController;
        rotationPIDController.enableContinuousInput(-Math.PI,Math.PI);       
        resetEncoders(); 

    }
    /**
     * 
     * @return Returns rotations of translation motor BEFORE GEAR RATIO
     */
    public double getTransPosition(){

        return transEncoder.getPosition(); 

    }

    /**
     * @return Returns rotations of rotation motor BEFORE GEAR RATIO
     */
    public double getRotPosition(){

        return rotEncoder.getPosition();

    }
    /**
     * @return returns RPM of Translation BEFORE GEAR RATIO
     */
    public double getTransVelocity(){
        
        return transEncoder.getVelocity(); 

    }
    /**
     * 
     * @return returns RPM of Rotation BEFORE GEAR RATIO
     */
    public double getRotVelocity(){

        return rotEncoder.getVelocity();

    }
    /**
     * Resets Relative encoder to Abs encoder position
     */
    public void resetEncoders(){

        rotEncoder.setPosition((universalEncoder.getAbsolutePosition()-universalEncoder.getPositionOffset())*18);

    }
    /**
     * 
     * @return a swerve module state object describing the current speed of the translation and rotation motors
     * @see SwerveModuleState
     */
    public SwerveModuleState getState(){

        // Returns SwerveModuleState

        return new SwerveModuleState(getTransVelocity()*Constants.RPMtoMPS*Constants.driveEncoderConversionFactortoRotations,
            new Rotation2d(getRotPosition()*Constants.angleEncoderConversionFactortoRad));
    
    }
    /**
     * Sets the motor speeds passed into constructor
     * @param desiredState takes in SwerveModule state
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState){
        
        //Stops returning to original rotation

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) 
        {
            stop();
            return;
        }

        //No turning motors over 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        //PID Controller for both translation and rotation
        transMotor.set(transController.calculate(
            transEncoder.getVelocity()*Constants.driveEncoderConversionFactortoRotations*Constants.RPMtoMPS,
            desiredState.speedMetersPerSecond)/Constants.maxSpeedMPS);
        
        //transMotor.set(desiredState.speedMetersPerSecond/Constants.maxSpeed);
        //Keep this
        
        rotMotor.set(rotationPIDController.calculate(rotEncoder.getPosition()*Constants.angleEncoderConversionFactortoRad,
            desiredState.angle.getRadians()));


    }
    /**
     * This is only for PID tuning
     * @param setPoint PID setpoint
     */
    public void updatePositions(double setPoint){

        //FOR PID TUNING ONLY

        rotationPIDController.setPID(Constants.kP, Constants.kI, Constants.kD);
        rotationPIDController.disableContinuousInput();
        double sp = rotationPIDController.calculate((getRotPosition()-0.5)*2*Math.PI/18, setPoint);
        rotMotor.set(sp);
    }


    /**
     * Call this in execute as it uses a PID controller
     */
    public void returnToOrigin(){

        //Sets wheel rot to original state

        System.out.println("In PID loop");
        rotMotor.set(rotationPIDController.calculate(((getRotPosition()%18)*2*Math.PI/18), 0));
        rotationPIDController.setTolerance(0);
    }

    /**
     * 
     * @return the total distance traveled by the module (Meters) and Rotation value (Rad) in the form of a SwerveModulePostion object
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getModulePos(){

        return new SwerveModulePosition(transEncoder.getPosition()*Constants.driveEncoderConversionFactortoRotations*Constants.kDriveEncoderRot2Meter,
            new Rotation2d(getRotPosition()*Constants.angleEncoderConversionFactortoRad));
    
    }
    /**
     * Stops the both motors
     */
    public void stop() {
        transMotor.set(0);
        rotMotor.set(0);

    }

    public PIDController getPIDController(){
        return this.rotationPIDController;
    }
    /**
     * Sets the mode of the translation motor
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeTrans(IdleMode mode){
        
        transMotor.setIdleMode(mode);
    }
    /**
     * Permanently burns settings into the sparks
     * @see setModeRot
     * @see setModeTrans
     */
    public void burnSparks(){
        rotMotor.burnFlash();
        transMotor.burnFlash();
    }
    /**
     * Sets the mode of the rotation motor
     * @param mode use a spark max idle mode (brake or coast)
     * @see IdleMode
     */
    public void setModeRot(IdleMode mode){
        rotMotor.setIdleMode(mode);
    }
}