// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
 
  //after starting command we continue forward until gyro detects roll of 15 degrees
  //then we start tracking the displacement in order to avoid overshooting.

  //reseting the gyro does not reset the axis of the robot
  //which means pitch and roll will always be relative

  float roll, pitch, yaw, displacement;
  Pose2d initialPose;
  Transform2d displacementTransform;

  Translation2d tilt;

  //arbritrary - fix later with dynamic speed control :)
  float[] speeds = {.25f, .35f, 5f};

  Input joyee;
  SwerveSubsystem swervee;

  AHRS navx;

  float percentagePower;

  float deadzone = 2.5f;

  boolean isOnChargingStation = false;

  

  //as of right now this command is not toggleable
  public AutoBalanceCommand(SwerveSubsystem swervee) {
    this.swervee = swervee;

    tilt = new Translation2d();

    addRequirements(swervee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roll = swervee.getRoll();
    pitch = swervee.getPitch();
    yaw = swervee.getYaw();

    this.swervee.setIdleModeForAll(IdleMode.kBrake,IdleMode.kBrake);
  }

  
  @Override
  public void execute() {
    
    roll = swervee.getRoll();
    pitch = swervee.getPitch();
    yaw = swervee.getYaw();

    goForwardSlowly();


    putDashboard();

  }

  void goForwardSlowly(){
    
    if(Math.abs(pitch) <= deadzone && isOnChargingStation){
      //robot is stable and on the carpet ground not charge station
      swervee.setMotors(0, speeds[0], 0);
    }
    else if(Math.abs(pitch) - 15 <= deadzone && !isOnChargingStation){
      isOnChargingStation = true;
      initialPose = this.swervee.getRobotPose();
    }


    if (isOnChargingStation){
      displacementTransform = this.swervee.getRobotPose().minus(initialPose);
      if(Math.abs(pitch) > 15){
        if(pitch < 0){
          percentagePower = -1;
        }else if(pitch > 0){
          percentagePower = 1;
        }
      }else{
        percentagePower = pitch / 15;
      }

      if(Math.abs(pitch) > 2.5){
        swervee.setMotors(0, percentagePower * speeds[0], 0);
      }else{
        //we dont
        swervee.stopAllAndBrake();
      }
    }



  }

  void putDashboard(){
    SmartDashboard.putNumber("Roll", roll);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putBoolean("IsOnDashboard", isOnChargingStation);
  }

  
  @Override
  public void end(boolean interrupted) {
    
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}