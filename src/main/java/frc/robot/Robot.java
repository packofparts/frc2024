// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  enum TestMode {
    DRIVE_MOTOR,
    TURN_MOTOR
  }

  private final Drivetrain _swerveDrivetrain = new Drivetrain();
  private final XboxController _xboxController = new XboxController(0);
  
  private final SlewRateLimiter _xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter _yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter _rotLimiter = new SlewRateLimiter(3);

  int[] testModules = {0, 1, 2, 3};

  // TestMode testMode = TestMode.TURN_MOTOR; 
  TestMode testMode = TestMode.DRIVE_MOTOR; 
  double testSpeed = .5;

  @Override
  public void autonomousPeriodic() {
    drive(false);
    _swerveDrivetrain.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    drive(true);
  }

  @Override
  public void testInit() {
    // CANSparkMaxLowLevel.enableExternalUSBControl(true);

    _swerveDrivetrain.stopMotors();
    _swerveDrivetrain.resetDriveEncoder(0);
    _swerveDrivetrain.resetSteeringEncoder(0);
  }
  
  @Override
  public void testPeriodic() {
    for (int i : testModules) {
      if (testMode == TestMode.DRIVE_MOTOR) {
        _swerveDrivetrain.setDriveMotorSpeed(i, testSpeed);
      } else {
        _swerveDrivetrain.setTurnMotorSpeed(i, testSpeed);
      }
      
      var v1 = _swerveDrivetrain.getDrivingEncoderPositionRaw(i);
      _swerveDrivetrain.debug(i, "DrivingEncoderPositionRaw", v1);

      var v2 = _swerveDrivetrain.getDrivingEncoderVelocityRaw(i);
      _swerveDrivetrain.debug(i, "DrivingEncoderVelocityRaw", v2);

      var v3 = _swerveDrivetrain.getRotationEncoderPositionRaw(i);
      _swerveDrivetrain.debug(i, "RotationEncoderPositionRaw", v3);

      double mps = _swerveDrivetrain.getModuleDrivingSpeedMPS(i);
      _swerveDrivetrain.debug(i, "ModuleDrivingSpeedMPS", mps);
      
      double distanceMeters = _swerveDrivetrain.getModulePositionMeters(i);
      _swerveDrivetrain.debug(i, "ModulePositionMeters", distanceMeters);
      
      double angleDeggrees = _swerveDrivetrain.getModulePositionAngleDeg(i);
      _swerveDrivetrain.debug(i, "ModulePositionAngleDeg", angleDeggrees);
    }
  }
  
  private void drive(boolean fieldRelative) {
    double xSpeed = MathUtil.applyDeadband(_xboxController.getLeftY(), 0.02);
    xSpeed = Drivetrain.kMaxSpeedMPS * -_xspeedLimiter.calculate(xSpeed);

    double ySpeed = MathUtil.applyDeadband(_xboxController.getLeftX(), 0.02);
    ySpeed = Drivetrain.kMaxSpeedMPS * -_yspeedLimiter.calculate(ySpeed);

    double rotSpeed = MathUtil.applyDeadband(_xboxController.getRightX(), 0.02);
    rotSpeed = Drivetrain.kMaxAngularSpeedRadPerSec * -_rotLimiter.calculate(rotSpeed);

    _swerveDrivetrain.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
  }
}