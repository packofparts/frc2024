package frc.robot;

import org.opencv.core.RotatedRect;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveModule {
    private CANSparkMax translationMotor;
    private CANSparkMax rotationMotor;

    private RelativeEncoder translationRelativeEncoder, rotationRelativeEncoder;

    double wheelRadius = 3;

    private PIDController translationPIDController = new PIDController(10, .0, 1);
    private PIDController rotationPIDController = new PIDController(1, 0.0, .25);


    private FlywheelSim translationSim;
    private FlywheelSim rotationSim;

    double translationPIDOutput, rotationPIDOutput, rotationEncoderPos, translationEncoderPosition, translationVelocity;

    
    private String nameOfModule;

    double previousTranslationP, previousTranslationI, previousTranslationD, previousRotationP, previousRotationI, previousRotationD;

    boolean doUpdatePID = false;

    public SwerveModule(int transMotorId, int rotMotorId, boolean transInverted, boolean rotInverted, String _name){
        this.nameOfModule = _name;
        
        translationMotor = new CANSparkMax(transMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotMotorId, MotorType.kBrushless);
        
        
        translationMotor.setInverted(transInverted); //shit dont do anything bro
        rotationMotor.setInverted(rotInverted); //dis shit also dont do anything


        translationRelativeEncoder = translationMotor.getEncoder();
        rotationRelativeEncoder = rotationMotor.getEncoder();

        rotationRelativeEncoder.setPositionConversionFactor(1);

        
        translationSim = new FlywheelSim(DCMotor.getNEO(1), 8, 5.5);
        rotationSim = new FlywheelSim(DCMotor.getNEO(1), 12, 5.5);


        
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }





    public double getTranslationPosition(){
    
        return translationRelativeEncoder.getPosition();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getTranslationPosition(), getRotation2d());
    }

    public double getRotPosition(){
        return rotationRelativeEncoder.getPosition() % 360;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getRotPosition());
    }

    public double getTransVelocity(){
        return translationVelocity / 0.02;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getTransVelocity(), Rotation2d.fromDegrees(getRotPosition()));
    }


    public void setDesiredState(SwerveModuleState desiredState){

        updatePIDValues();


        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(Units.degreesToRadians(rotationRelativeEncoder.getPosition() % 360)));

        translationPIDOutput = translationPIDController.calculate(translationVelocity, desiredState.speedMetersPerSecond / 18);
        rotationPIDOutput = rotationPIDController.calculate(Units.degreesToRadians(rotationRelativeEncoder.getPosition() % 360), desiredState.angle.getRadians());

        SmartDashboard.putString(nameOfModule, Math.floor(rotationRelativeEncoder.getPosition() % 360) + " " + desiredState.angle.getDegrees() + " " + rotationPIDOutput);
        SmartDashboard.putString(nameOfModule, Math.floor(rotationRelativeEncoder.getPosition() % 360) + " " + desiredState.angle.getDegrees() + " " + rotationPIDOutput);

        translationMotor.set(translationPIDOutput);
        
        rotationMotor.set(rotationPIDOutput);
        
        simulationPeriodic(0.02);
    }

    public void setDesiredState(State desiredShit, Rotation2d desiredRotation){
        updatePIDValues();

        translationPIDOutput = translationPIDController.calculate(translationVelocity, desiredShit.velocityMetersPerSecond / 18);
        rotationPIDOutput = rotationPIDController.calculate(Units.degreesToRadians(rotationRelativeEncoder.getPosition() % 360), desiredRotation.getRadians());

        translationMotor.set(translationPIDOutput);
        rotationMotor.set(rotationPIDOutput);

        simulationPeriodic(0.02);
    }

    public void simulationPeriodic(double dt){
        translationSim.setInputVoltage(translationPIDOutput * RobotController.getInputVoltage());
        rotationSim.setInput(rotationPIDOutput * RobotController.getInputVoltage());

        translationSim.update(dt);
        rotationSim.update(dt);

        translationEncoderPosition += translationSim.getAngularVelocityRPM() / 60 * wheelRadius * 2 * Math.PI * dt;
        translationRelativeEncoder.setPosition(translationEncoderPosition);
        translationVelocity = translationSim.getAngularVelocityRPM() / 60 * wheelRadius * 2 * Math.PI * dt;

        rotationEncoderPos += rotationSim.getAngularVelocityRPM() / 60 * 360 * dt;
        rotationRelativeEncoder.setPosition(rotationEncoderPos);
    }


    public PIDController getPIDController(){
        return this.rotationPIDController;
    }

    public void stop() {
        translationMotor.set(0);
        rotationMotor.set(0);
    }

    public double metersToRadians(double meters, double radius){
        return meters / radius;
    }

    public void updatePIDValues(){
        if(!doUpdatePID){
            return;
        }

        translationPIDController.setP(DriveSubsystem.translationP);
        translationPIDController.setI(DriveSubsystem.translationI);
        translationPIDController.setD(DriveSubsystem.translationD);


        rotationPIDController.setP(DriveSubsystem.rotationP);
        rotationPIDController.setI(DriveSubsystem.rotationI);
        rotationPIDController.setD(DriveSubsystem.rotationD);

    }
}
