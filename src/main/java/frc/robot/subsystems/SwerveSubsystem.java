/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.commands.AutoBalanceCommand;


public class SwerveSubsystem extends SubsystemBase {
  //Bevel Gear must be facing to the left in order to work
  private final SwerveModule frontLeft = new SwerveModule(Constants.frontLeftDrive, Constants.frontLeftSteer, 0,false, true,1,false, true,Constants.flPID,Constants.flPIDTrans);
  private final SwerveModule frontRight = new SwerveModule(Constants.frontRightDrive, Constants.frontRightSteer,1,true,true,0.805,false, true,Constants.frPID,Constants.frPIDTrans);
  private final SwerveModule backLeft = new SwerveModule(Constants.rearLeftDrive, Constants.rearLeftSteer,2,true,true,0.156,false, true,Constants.blPID,Constants.flPIDTrans);
  private final SwerveModule backRight = new SwerveModule(Constants.rearRightDrive, Constants.rearRightSteer,3,true,true,0.801,false, true, Constants.brPID,Constants.brPIDTrans); 
  private final PIDController headingController;

  public SwerveDriveKinematics m_kinematics;
  private ChassisSpeeds chassisSpeeds1;
  public SwerveDriveOdometry m_odometry;
  AHRS navx = new AHRS(Port.kMXP);
  SwerveModule [] rawMods;

  public SwerveSubsystem() {
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2));
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d(), this.getModuleStates());
    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
    resetGyro();

    headingController = new PIDController(0.5, 0, 0);

    resetRobotPose(new Pose2d());
    rawMods = getRawModules();

  }

  @Override
  public void periodic() {
    for(int i = 0; i<getRawModules().length;i++){
      SmartDashboard.putNumber("RelativeEnc"+i, getRawModules()[i].getRotPosition());
      SmartDashboard.putNumber("TruePos"+i, getRawModules()[i].universalEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Generic"+i, getRawModules()[i].universalEncoder.getAbsolutePosition()-getRawModules()[i].universalEncoder.getPositionOffset());
      SmartDashboard.putNumber("Offset"+i, getRawModules()[i].universalEncoder.getPositionOffset());
    }

    m_odometry.update(getRotation2d(), getModuleStates());
    if(Input.resetGyro()){resetGyro();}
    if(Input.runAutoBalance()){
      AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(this);
      autoBalanceCommand.schedule();
    }
  }
  public void resetGyro(){
    navx.reset();
  }
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}
  public SwerveModulePosition[] getModuleStates(){
    return(new SwerveModulePosition[]{frontLeft.getModulePos(),frontRight.getModulePos(),backLeft.getModulePos(),backRight.getModulePos()});
  }
  public void setMotors(double x,double y, double rot){
    rot = headingController.calculate(navx.getRate(), rot);
    if (!Input.getRobotOriented()){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x,y, rot);}
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);
    this.setModuleStates(moduleStates);
  }


  public void resetRobotPose(Pose2d pose){
    m_odometry.resetPosition(pose.getRotation(), getModuleStates(), pose);
  }

  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }
  public void goToOrigin(){
    System.out.println("executed");
    for (SwerveModule mod : rawMods){
      mod.returnToOrigin();
    }
  }

  public SwerveModule[] getRawModules(){
    return new SwerveModule[]{frontLeft,frontRight,backLeft,backRight};
  }
  public void stopAllAndBrake(){
    for (SwerveModule mod : rawMods){
      mod.stop();
    }
  }

  public void setIdleModeForAll(IdleMode transMode,IdleMode rotMode){
    for (SwerveModule mod : rawMods){
      mod.setModeRot(rotMode);
      mod.setModeTrans(transMode);
    }
  }

  public float getRoll(){
    return this.navx.getRoll();
  }

  public float getYaw(){
    return this.navx.getYaw();
  }

  public float getPitch(){
    return this.navx.getPitch();
  }

}