// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CompConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveMode;

public class AutoBalanceCommand extends CommandBase {

  //detect angle, angular velocity, and position
  //go forward until angle is about 11 degrees (which means it is on the charging station)

  //IDEAS:
  //feed angle into pid and output position 
  //maybe use feedforward to beat gravity to make the pid more accurate
  //if angular velocity increases drastically, slow down
  
  private SwerveSubsystem swerveSubsystem;

  
  private boolean isOnChargingStation = false;
  private boolean isAtEnd = false;

  private float prevPitch, pitch, roll, yaw, pitchSpeed;
  private double initXPos, currentXPos;
  
  private PIDController velocityController, velocityController2, positionController;
  private SimpleMotorFeedforward xXGravityDestroyer420Xx;

  private double lasttime, currenttime, deltatime;
  private Timer timer;


  


  public AutoBalanceCommand(SwerveSubsystem swervee) {
    this.swerveSubsystem = swervee;

    velocityController = CompConstants.velocityController;
    velocityController.setTolerance(CompConstants.kAngleDeadZoneDeg);
    

    positionController = new PIDController(.5, 0, 0);

    timer = new Timer();

    addRequirements(swervee);
  }


  @Override
  public void initialize() {

    isOnChargingStation = false;
    prevPitch=0; pitch=0; roll=0; yaw=0; pitchSpeed = 0;
    initXPos = 0;
    currentXPos = 0;
    isAtEnd = false;
    lasttime = 0;
    currenttime=0;
    deltatime=0;

    

    this.initXPos = this.swerveSubsystem.getRobotPose().getX();
    this.currentXPos = this.initXPos;


    timer.start();

    //maybe rotate it 90 degrees or just rotate it so that it
    //so that the center of gravity is closer to the balance point
  }

  
  @Override
  public void execute() {

    this.lasttime = this.currenttime;
    this.currenttime = timer.get();
    this.deltatime = this.currenttime - this.lasttime;


    this.prevPitch = this.pitch;
    this.pitch = this.swerveSubsystem.getPitch();
    this.roll = this.swerveSubsystem.getRoll();
    this.yaw = this.swerveSubsystem.getYaw();
    this.pitchSpeed = (this.pitch - this.prevPitch) / (float) this.deltatime;

    this.currentXPos = swerveSubsystem.getRobotPose().getX();

    if (!this.isOnChargingStation){
      this.goForwardUntilOnChargeStation();
    }else if (Math.abs(this.roll) >= 2.5){
      this.doBalanceMethod1();
      //this.doBalanceMethod2();
    }else{
      //brake
      this.swerveSubsystem.stopAllAndBrake();
      //turn wheels to X positions
    }
  }

  //only uses PID control
  private void doBalanceMethod1(){
    double pidOutput = velocityController.calculate(this.roll, 0);
    SmartDashboard.putNumber("BalancePIDOutput", pidOutput);


    this.swerveSubsystem.setMotors(pidOutput/9, 0, 0,DriveMode.AUTO,false);
  }

  //incorporates angular velocity
  //if angular velocity is too high, then the station must be turning
  
  private void doBalanceMethod2(){
    double pidOutput = velocityController.calculate(this.roll, 0);
    SmartDashboard.putNumber("BalancePIDOutput", pidOutput);

    //detect change in velocity over threshold and stop until velocity is down again
    if(Math.abs(this.pitchSpeed) >= CompConstants.pitchSpeedThreshold){
      // this.swerveSubsystem.stopAllAndBrake();
      this.swerveSubsystem.setMotors(-pidOutput/8, 0, 0,DriveMode.AUTO,false);
      //maybe slightly move back for a tiny bit
    }else{
      

      this.swerveSubsystem.setMotors(pidOutput/13, 0, 0,DriveMode.AUTO,false);
    }
  }

  private void goForwardUntilOnChargeStation(){
    if(Math.abs(this.roll) >= CompConstants.onChargeStationOrientation){
      this.isOnChargingStation = true;
      this.initXPos = this.swerveSubsystem.getRobotPose().getX();
      return;
    }

    this.swerveSubsystem.setMotors(CompConstants.entrySpeed, 0, 0,DriveMode.AUTO,false);

  }


  private void checkFallOff(){
    this.currentXPos = this.swerveSubsystem.getRobotPose().getX();
    double difference = (this.currentXPos - this.initXPos) * 100; //to centimeters

    if(difference >= FieldConstants.kChargeStationBalanceBeamLengthCm - DriveConstants.kTrackWidthMeters - .5){
      this.isAtEnd = true;
    }else{
      this.isAtEnd = false;
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}