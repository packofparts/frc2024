package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {

    private int m_MotorTransID;
    private int m_MotorRotID;
    private int m_UniversalEncoderID;
    private CANSparkMax transMotor;
    private CANSparkMax rotMotor;
    private RelativeEncoder transEncoder;
    private RelativeEncoder rotEncoder;
    private AnalogInput universalEncoder;
    public SparkMaxPIDController rotPID;
    public PIDController rotationPIDTest;
    private Boolean isAbsoluteEncoder;
    private double universalEncoderOffset;
    private Boolean m_transInverted;
    private Boolean m_rotInverted;
    private Boolean encoderInverted;
    

    public SwerveModule(int motorTransID, int motorRotID, int universalEncoderID,
     Boolean transInverted, Boolean rotInverted, double universalEncoderOffsetinit,
     Boolean universalEncoderInverted, boolean isAbsEncoder){
        this.encoderInverted = universalEncoderInverted;
        this.isAbsoluteEncoder=isAbsEncoder;
        this.m_MotorTransID = motorTransID;
        this.m_UniversalEncoderID = universalEncoderID;
        this.m_MotorRotID = motorRotID;
        this.m_transInverted = transInverted;
        this.m_rotInverted = rotInverted;
        this.universalEncoderOffset = universalEncoderOffsetinit;
        
        transMotor = new CANSparkMax(this.m_MotorTransID, MotorType.kBrushless);
        
        rotMotor = new CANSparkMax(this.m_MotorRotID, MotorType.kBrushless);
        if (isAbsEncoder){
            universalEncoder = new AnalogInput(this.m_UniversalEncoderID); //basically does shit
        }


        transMotor.setInverted(this.m_transInverted);
        rotMotor.setInverted(this.m_rotInverted);


        transEncoder = transMotor.getEncoder();
        rotEncoder = rotMotor.getEncoder();
        
        resetEncoders();
        rotPID = rotMotor.getPIDController();
        rotationPIDTest = new PIDController(0.1, 0, 0);
        rotationPIDTest.enableContinuousInput(-Math.PI,Math.PI);
        
    }
    public double getTransPosition(){
        return transEncoder.getPosition(); 
    }
    public double getRotPosition(){
        return rotEncoder.getPosition();
    
    }
    public double getTransVelocity(){
        return transEncoder.getVelocity();
    }
    public double getRotVelocity(){
        return rotEncoder.getVelocity();
    }
    public double getUniversalEncoderRad(){
        if (isAbsoluteEncoder) {
            double angle = universalEncoder.getVoltage()/RobotController.getVoltage5V();
            angle*=2.0 * Math.PI;
            angle-= universalEncoderOffset;
            if (this.encoderInverted){
                return angle*-1;
            }
            else{
                return angle;
             }
            
        }
        return 0;
        
    }
    public void resetEncoders(){
        transEncoder.setPosition(0);
        rotEncoder.setPosition(0);// 

        //hello - 8/3/22
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getTransVelocity(),new Rotation2d(getRotPosition()*2*Math.PI/18));
    }
    public void setDesiredState(SwerveModuleState desiredState){
        
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) 
        {
            stop();
            return;
        }

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        SmartDashboard.putNumber("RotationPosition"+this.m_MotorRotID, getRotPosition());
        SmartDashboard.putNumber("DesiredState"+this.m_MotorRotID, desiredState.angle.getRadians());
       // if (this.m_transInverted){transMotor.set(-desiredState.speedMetersPerSecond/Constants.maxSpeed);}
        transMotor.set(desiredState.speedMetersPerSecond/Constants.maxSpeed);
        rotMotor.set(rotationPIDTest.calculate(rotEncoder.getPosition()*Constants.angleEncoderConversionFactor, desiredState.angle.getRadians()));
        //System.out.println("setPoint is: "+ getRotPosition());
    }
    public void updatePositions(Double setPoint){
        rotationPIDTest.setPID(Constants.kP, Constants.kI, Constants.kD);
        rotationPIDTest.disableContinuousInput();
        double sp = rotationPIDTest.calculate(rotEncoder.getPosition()*2*Math.PI/18, setPoint);
        //System.out.println(sp);
        rotMotor.set(sp);
    }
    public void returnToOrigin(){
        System.out.println("In PID loop");
        rotMotor.set(rotationPIDTest.calculate(rotEncoder.getPosition()*2*Math.PI/18, 0));
        rotationPIDTest.setTolerance(0);
    }
    public SwerveModulePosition getModulePos(){
        return new SwerveModulePosition(transEncoder.getPosition()*Constants.kDriveEncoderRPM2MeterPerSec,new Rotation2d(getRotPosition()));
    }
    public void stop() {
        transMotor.set(0);
        rotMotor.set(0);

    }
    public PIDController getPIDController(){
        return this.rotationPIDTest;
    }
    public void setPidController(double p, double i, double d){
        rotationPIDTest.setP(p);
        rotationPIDTest.setI(i);
        rotationPIDTest.setD(d);
    }


}