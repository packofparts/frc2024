package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    
    private static final double kDriveMotorMaxSpeedMeterPerSecond = 3.0;
    private static final double kDriveEncoderRotations2Meters = Math.PI * SwerveConfig.Const.kWheelDiameterMeters;    
    private static final double kDriveGearRatio = 1.0/10; // TODO Check ratio

    public static final double kDrivingEncoderPositionConversionFactor = kDriveEncoderRotations2Meters*kDriveGearRatio;
    public static final double kDriveEncoderRPM2MetersPerSec = kDriveEncoderRotations2Meters*kDriveGearRatio / 60;

    public static final double kAngularEncoderConversionFactor = 2 * Math.PI * 1.0/18;

    private final SwerveConfig _config;
    
    private final CANSparkMax _drivingMotor;
    private final RelativeEncoder _drivingEncoder;

    private final CANSparkMax _steeringMotor;
    private final RelativeEncoder _steeringRelativeEncoder;    
    private final CANCoder _steeringAbsEncoder;

    private final PIDController _drivingPIDController;
    private final PIDController _steeringPIDController;

    public SwerveModule(SwerveConfig config) {
        _config = config;
        _drivingMotor = new CANSparkMax(config.driveMotorCanID, MotorType.kBrushless);
        _drivingMotor.setInverted(config.driveMotorInverse);
        _drivingEncoder = _drivingMotor.getEncoder();

        _steeringMotor = new CANSparkMax(config.rotationMotorCanID, MotorType.kBrushless);
        _steeringMotor.setInverted(config.rotationMotorInverse);
        _steeringRelativeEncoder = _steeringMotor.getEncoder();
        _steeringAbsEncoder = new CANCoder(config.rotationEncoderCanID);

        _drivingPIDController = config.drivePIDController;
        _steeringPIDController = config.rotationPIDController;        
        _steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        burnSparks();
    }
    
    private void burnSparks() {
        _drivingMotor.burnFlash();
        _steeringMotor.burnFlash();
    } 

    public String getName() {
        return _config.moduleName;
    }

    public double getDrivingEncoderPositionRaw() {
        return _drivingEncoder.getPosition();
    }

    public double getDrivingEncoderVelocityRaw() {
        return _drivingEncoder.getVelocity();
    }
    
    public double getRotationEncoderPositionRaw() {
        return _steeringRelativeEncoder.getPosition();
    }
    
    public double getDrivingEncoderPositionMeters() {
        return _drivingEncoder.getPosition() * kDrivingEncoderPositionConversionFactor; 
    }

    public double getDrivingEncoderSpeedMetersPerSecond() {
        return _drivingEncoder.getVelocity() * kDriveEncoderRPM2MetersPerSec; 
    }
    
    public double getRotationEncoderPositionRadians(){
        return _steeringRelativeEncoder.getPosition() * kAngularEncoderConversionFactor;
    }

    public Rotation2d getRotation2d(){
        double steeringRadians = getRotationEncoderPositionRadians();
        return Rotation2d.fromRadians(steeringRadians);
    }

    public SwerveModuleState getState() {
        double driveSpeedMPS = getDrivingEncoderSpeedMetersPerSecond();
        return new SwerveModuleState(driveSpeedMPS, getRotation2d());
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = getDrivingEncoderPositionMeters();
        return new SwerveModulePosition(distanceMeters, getRotation2d());
    }
    
    /**
     * Sets the desired state for the module.
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        debug("state_MPS_before_optimize", desiredState.speedMetersPerSecond);
        debug("state_radians_before_optimize", desiredState.angle.getRadians());
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation2d());
        debug("state_MPS_after_optimize", desiredState.speedMetersPerSecond);
        debug("state_after_before_optimize", desiredState.angle.getRadians());
        
        double driveOutput = state.speedMetersPerSecond / kDriveMotorMaxSpeedMeterPerSecond;
        debug("SetDrivingMotorOutput", driveOutput);
        _drivingMotor.set(driveOutput);

        double turnOutput = _steeringPIDController.calculate(getRotationEncoderPositionRadians(), desiredState.angle.getRadians());
        debug("SetTurnMotorOutput", turnOutput);
        _steeringMotor.set(turnOutput);
    }

    public void resetDriveEncoder(double rotation) {
        _drivingEncoder.setPosition(rotation);
    }

    public void resetSteeringEncoder(double rotation) {
        _steeringRelativeEncoder.setPosition(rotation);
    }

    public void stop() {
        _drivingMotor.stopMotor();
        _steeringMotor.stopMotor();
    }

    public void setDriveMotorSpeed(double speed) {
        _drivingMotor.set(speed);
    }

    public void setSteerMotorSpeed(double speed) {
        _steeringMotor.set(speed);
    }

    public void debug(String s, double v) {
        SmartDashboard.putNumber(_config.moduleName + "-" + s, v);
    }
}