/*----------------------------------------------------------------------------*/
/* Copyright (c) 2023 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutonTrajectory;


public class SwerveSubsystem extends SubsystemBase {
  //Bevel Gear must be facing to the left in order to work

  private final PIDController headingController;
  private final SwerveModule frontLeft = new SwerveModule(Constants.frontLeftDrive, Constants.frontLeftSteer, 0,false, true,.997,false, true,Constants.flPID, Constants.flPIDTrans);
  private final SwerveModule frontRight = new SwerveModule(Constants.frontRightDrive, Constants.frontRightSteer,1,true,true,0.875,false, true,Constants.frPID, Constants.frPIDTrans);
  private final SwerveModule backLeft = new SwerveModule(Constants.rearLeftDrive, Constants.rearLeftSteer,2,false,true,0.350,false, true,Constants.blPID, Constants.blPIDTrans);
  private final SwerveModule backRight = new SwerveModule(Constants.rearRightDrive, Constants.rearRightSteer,3,true,true,0.803,false, true, Constants.brPID, Constants.brPIDTrans); 

  private SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);


  AutonTrajectory at;

  public SwerveDriveKinematics m_kinematics;
  private ChassisSpeeds chassisSpeeds1;
  private SwerveDriveOdometry m_odometry;
  AHRS navx = new AHRS(Port.kMXP);
  private Joysticks joy;
  SwerveModule [] rawMods;

  public SwerveSubsystem(Joysticks joys) {
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2));
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation2d(), this.getModuleStates());
    //m_estimator = new SwerveDrivePoseEstimator(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs)
    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
    this.joy = joys;
    resetGyro();

    headingController = new PIDController(0.5, 0, 0);

    resetRobotPose();
    rawMods = getRawModules();
    navx.calibrate();

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Calibration", navx.isCalibrating());
    SmartDashboard.putBoolean("DoneCalibration",navx.isMagnetometerCalibrated());
    for(int i = 0; i<getRawModules().length;i++){
      SmartDashboard.putNumber("RelativeEnc"+i, getRawModules()[i].getRotPosition());
      SmartDashboard.putNumber("TruePos"+i, getRawModules()[i].universalEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Generic"+i, getRawModules()[i].universalEncoder.getAbsolutePosition()-getRawModules()[i].universalEncoder.getPositionOffset());
      SmartDashboard.putNumber("Offset"+i, getRawModules()[i].universalEncoder.getPositionOffset());
    }

    m_odometry.update(getRotation2d(), getModuleStates());
    if(joy.resetGyro()){resetGyro();}
    if(joy.trajectoryRun()){
      at = new AutonTrajectory(this, 0);
      at.schedule();
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

    if(true){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kTeleDriveMaxSpeedMetersPerSecond*10);
      SmartDashboard.putNumber("Module1ROT", desiredStates[0].angle.getRadians());
      SmartDashboard.putNumber("Module2ROT", desiredStates[1].angle.getRadians());
      SmartDashboard.putNumber("Module3ROT", desiredStates[2].angle.getRadians());
      SmartDashboard.putNumber("Module4ROT", desiredStates[3].angle.getRadians());
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      backLeft.setDesiredState(desiredStates[2]);
      backRight.setDesiredState(desiredStates[3]);
    }
}
  public SwerveModulePosition[] getModuleStates(){
    return(new SwerveModulePosition[]{frontLeft.getModulePos(),frontRight.getModulePos(),backLeft.getModulePos(),backRight.getModulePos()});
  }
  public void setMotors(double x,double y, double rot){
    if (navx.getRate() == 0 & rot == 0 & Constants.gyroHold == true){
      rot = Constants.brPID.calculate(navx.getAngle(),Constants.priorGyroAngle);
    }

    if (!joy.getRobotOriented()){
      chassisSpeeds1 = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getRotation2d());
    } else {chassisSpeeds1 = new ChassisSpeeds(x,y, rot);}

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds1);
    this.setModuleStates(moduleStates);
    Constants.priorGyroAngle = navx.getAngle();
    SmartDashboard.putNumber("Module1CurrentROT",frontLeft.getRotPosition());
    SmartDashboard.putNumber("Module2CurrentROT", frontRight.getRotPosition());
    SmartDashboard.putNumber("Module3CurrentROT", backLeft.getRotPosition());
    SmartDashboard.putNumber("Module4CurrentROT", backRight.getRotPosition());
    SmartDashboard.putNumber("ChassisSpeeds POT", chassisSpeeds1.omegaRadiansPerSecond);
    SmartDashboard.putNumber("ChassisSpeed X", chassisSpeeds1.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed Y", chassisSpeeds1.vyMetersPerSecond);
    SmartDashboard.putNumber("Heading", getHeading());
  }

  public void doFeedforwardAllMotors(double distanceMeters) {
    frontLeft.transController.setSetpoint(distanceMeters);
    double voltage = feedforwardController.calculate(5);
    frontLeft.transMotor.setVoltage(voltage);
    frontRight.transMotor.setVoltage(voltage);
    backLeft.transMotor.setVoltage(voltage);
    backRight.transMotor.setVoltage(voltage);
    while (frontLeft.transController.atSetpoint()) {
      frontLeft.transMotor.setVoltage(0);
      frontRight.transMotor.setVoltage(0);
      backLeft.transMotor.setVoltage(0);
      backRight.transMotor.setVoltage(0);
    }
  }

  public double[] getVelocity(){
    return null;
  }

  public void resetRobotPose(){
    m_odometry.resetPosition(getRotation2d(), getModuleStates(), getRobotPose());
  }

  public Pose2d getRobotPose(){
    return m_odometry.getPoseMeters();
  }

  public void goToOrigin(){
    System.out.println("executed");
    frontRight.returnToOrigin();
    frontLeft.returnToOrigin();
    backLeft.returnToOrigin();
    backRight.returnToOrigin();
  }

  public void setAllPIDControllers(double p, double i, double d) {
    System.out.println(p+" "+i+" "+d);
    frontRight.setPidController(p, i, d);
    frontLeft.setPidController(p, i, d);
    backRight.setPidController(p, i, d);
    backLeft.setPidController(p, i, d);
  }
  public SwerveModule[] getRawModules(){
    return new SwerveModule[]{frontLeft,frontRight,backLeft,backRight};
  }

}