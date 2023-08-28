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

  private final Drivetrain _swerveDrivetrain = new Drivetrain();
  private final XboxController _xboxController = new XboxController(0);
  
  private final SlewRateLimiter _xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter _yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter _rotLimiter = new SlewRateLimiter(3);

  int[] testModules = {0, 1, 2, 3};
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
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
    _swerveDrivetrain.stopMotors();
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