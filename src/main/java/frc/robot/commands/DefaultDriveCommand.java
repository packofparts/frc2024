// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends CommandBase {
  
  private final SwerveSubsystem mSwerve;
  private boolean mIsPrecisionToggle = false;
  
  public DefaultDriveCommand(SwerveSubsystem swerve) {
    mSwerve = swerve;
    addRequirements(mSwerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // x, y, and rot are inverted because of the Joystick configurations
    double x = -Input.getJoystickY();
    double y = -Input.getJoystickX();
    double rot = -Input.getRot();

    if (Input.resetGyro()){mSwerve.resetGyro();}

    if(Input.resetOdo()){mSwerve.resetOdometry();}

    if (Input.getPrecisionToggle()){mIsPrecisionToggle = !mIsPrecisionToggle;}

    if (mIsPrecisionToggle){
      x = x/3;
      y = y/3;
      rot = rot/6;

      x = Math.abs(x) > 0.04 ? x : 0.0;
      y = Math.abs(y) > 0.04 ? y :0.0;
      rot = Math.abs(rot) > 0.02 ? rot : 0.0;
    } else{
      x = Math.abs(x) > 0.10 ? x : 0.0;
      y = Math.abs(y) > 0.10 ? y : 0.0;
      rot = Math.abs(rot) > 0.15 ? rot : 0.0;
    }
    
    x *= SwerveConstants.TELE_MAX_SPEED_MPS;
    y *= SwerveConstants.TELE_MAX_SPEED_MPS;
    rot *= SwerveConstants.TELE_MAX_ROT_SPEED_RAD_SEC;
    
    mSwerve.setMotors(x, y, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // For now we are keeping this empty to handle interruptions in the future
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
