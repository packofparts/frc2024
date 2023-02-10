/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI.Port;
import com.revrobotics.CANSparkMax.IdleMode;
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
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2));
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d(), this.getModulePositions());
    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
    resetGyro();

    headingController = new PIDController(0.5, 0, 0);

    resetRobotPose(new Pose2d());
    rawMods = getRawModules();
    setIdleModeForAll(IdleMode.kCoast, IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    for(int i = 0; i<getRawModules().length;i++){
      SmartDashboard.putNumber("RelativeEnc"+i, getRawModules()[i].getRotPosition());
      SmartDashboard.putNumber("TruePos"+i, getRawModules()[i].universalEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Generic"+i, getRawModules()[i].universalEncoder.getAbsolutePosition()-getRawModules()[i].universalEncoder.getPositionOffset());
      SmartDashboard.putNumber("Offset"+i, getRawModules()[i].universalEncoder.getPositionOffset());
    }

    m_odometry.update(getRotation2d(), getModulePositions());
    if(Input.resetGyro()){resetGyro();}
    if(Input.runAutoBalance()){
      AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(this);
      autoBalanceCommand.schedule();
    }

    


  }
  /**
   * Sets the current YAW heading as the 0'd heading
   */
  public void resetGyro(){
    navx.reset();
  }

  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0)
   * @return the degrees at which the gyro is at
   */
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  /**
   * This gets the Rotation2d of the gyro (which is in continuous input)
   * @return the Rotation2d of the gyro
   * @see Rotation2d
   */
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
  /**
   * This function sets the current speeds of the swerve modules to the following array pattern
   * [frontleft, frontright, backleft, backright]
   * @see SwerveModuleState
   * @param desiredStates requires a SwerveModuleState array
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}
  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft, backright]
   * @see SwerveModulePosition
   */
  public SwerveModulePosition[] getModulePositions(){
    return(new SwerveModulePosition[]{frontLeft.getModulePos(),frontRight.getModulePos(),backLeft.getModulePos(),backRight.getModulePos()});
  }
  /**
   * 
   * @param x this is the forward velocity in meters/second
   * @param y this is the sideways velocity in meter/second (left is positive)
   * @param rot this is in radians/second counterclockwise
   * 
   * @apiNote Keep in mind all of this is field relative so resetting the gyro midmatch will also reset these params
   */
  public void setMotors(double x,double y, double rot){
    rot = headingController.calculate(navx.getRate(), rot);
    if (!Input.getRobotOriented()){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x,y, rot);}
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);
    this.setModuleStates(moduleStates);
  }



  /**
   * This method resets the pose of the robot to the desired robot pose
   * @param pose provide the new desired pose of the robot
   * @see Pose2d
   */
  public void resetRobotPose(Pose2d pose){
    m_odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }
  /**
   * @return provide the pose of the robot in meters
   */
  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }
  /**
   * makes all modules return to its original state
   */
  public void goToOrigin(){
    System.out.println("executed");
    for (SwerveModule mod : rawMods){
      mod.returnToOrigin();
    }
  }
  /**
   * @return an array of all the SwerveModule objects in the format [frontLeft,frontRight,backLeft,backRight]
   */
  public SwerveModule[] getRawModules(){
    return new SwerveModule[]{frontLeft,frontRight,backLeft,backRight};
  }
  /**
   * stops all swerve module rotation and translation
   */
  public void stopAllAndBrake(){
    for (SwerveModule mod : rawMods){
      mod.stop();
    }
  }
  /**
   * sets the Idle mode for all the modules
   * @param transMode idle mode for translation motor
   * @param rotMode idle mode for rotation motor
   * @see IdleMode
   */
  public void setIdleModeForAll(IdleMode transMode,IdleMode rotMode){
    for (SwerveModule mod : rawMods){
      mod.setModeRot(rotMode);
      mod.setModeTrans(transMode);
      mod.burnSparks();
    }
  }

  /**
   * forms an X shape with the wheels for grip
   */
  public void setXShape(){
    
  }

  /**
   * 
   * @return returns roll in degrees (180,-180)
   */
  public float getRoll(){
    return this.navx.getRoll();
  }
  /**
   * 
   * @return returns yaw in degrees (180,-180)
   */
  public float getYaw(){
    return this.navx.getYaw();
  }
  /**
   * 
   * @return returns pitch in degrees (180,-180)
   */
  public float getPitch(){
    return this.navx.getPitch();
  }

}