package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;

public class Drivetrain {
    public static final double kMaxSpeedMPS = 3.0; 

    // Half rotation per second
    public static final double kMaxAngularSpeedRadPerSec = Math.PI;
      
    private final SwerveModule _frontLeft = new SwerveModule(SwerveConfig.FRONT_LEFT);
    private final SwerveModule _frontRight = new SwerveModule(SwerveConfig.FRONT_RIGHT);
    private final SwerveModule _backLeft = new SwerveModule(SwerveConfig.BACK_LEFT);
    private final SwerveModule _backRight = new SwerveModule(SwerveConfig.BACK_RIGHT);
    
    public final SwerveModule[] modules = {_frontLeft, _frontRight, _backLeft, _backRight};
    private final AHRS _navx = new AHRS(Port.kMXP);
    
    private final SwerveDriveKinematics _kinematics = SwerveConfig.Const.kKinematics;
    
    private final SwerveDriveOdometry _odometry = new SwerveDriveOdometry(
        _kinematics,
        _navx.getRotation2d(),
        new SwerveModulePosition[] 
        {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        }
    );

    public Drivetrain() {
        _navx.reset();
    }

    public void setDriveMotorSpeed(int moduleId, double speed) {
        modules[moduleId].setDriveMotorSpeed(speed);
    }

    public void setTurnMotorSpeed(int moduleId, double speed) {
        modules[moduleId].setSteerMotorSpeed(speed);
    }

    public void stopMotors() {
        for (var module : modules) {
            module.stop();
        }
    }

    public void resetDriveEncoder(double v) {
        for (var module : modules) {
            module.resetDriveEncoder(v);
        }
    }

    public void resetSteeringEncoder(double v) {
        for (var module : modules) {
            module.resetSteeringEncoder(v);
        }
    }

    public double getDrivingEncoderPositionRaw(int id) {
        return modules[id].getDrivingEncoderPositionRaw();
    }

    public double getDrivingEncoderVelocityRaw(int id) {
        return modules[id].getDrivingEncoderVelocityRaw();
    }
    
    public double getRotationEncoderPositionRaw(int id) {
        return modules[id].getRotationEncoderPositionRaw();
    }

    public double getModulePositionMeters(int id) {
        return modules[id].getPosition().distanceMeters;
    }

    public double getModulePositionAngleRadians(int id) {
        return modules[id].getPosition().angle.getRadians();
    }

    public double getModulePositionAngleDeg(int id) {
        return modules[id].getPosition().angle.getDegrees();
    }

    public double getModuleDrivingSpeedMPS(int id) {
        return modules[id].getState().speedMetersPerSecond;
    }
    
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, _navx.getRotation2d());            
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        swerveModuleStates = _kinematics.toSwerveModuleStates(chassisSpeeds);                          
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeedMPS);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }
  
    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        _odometry.update(_navx.getRotation2d(),
            new SwerveModulePosition[] 
            {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()
            }
        );
    }

    public void debug(int id, String s, double v) {
        modules[id].debug(s, v);
    }
}