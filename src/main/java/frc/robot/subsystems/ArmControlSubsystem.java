
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CompConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.Util;


public class ArmControlSubsystem extends SubsystemBase {
  
  public static enum ArmSetting {
    NODE3,
    NODE2,
    NODE1,
    GNODE,
    SUBSTATION,
    NEUTRAL,

  }

  public static enum ArmMotorMode {
    COAST,
    BRAKE,
    OFF
  }

  public static enum MoveArmConfig {
    SIMULTANEOUS,
    SEQ_EXTEND_THEN_PIVOT,
    SEQ_PIVOT_THEN_EXTEND
  }
  

  boolean isCoast = false;
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.rightArmPivot);

  
  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();

  

  double initialPivotEncoderOffset;
  double currentPivotRotation = ArmConstants.zeroAngleRad;
  double desiredPivotRotation = ArmConstants.minAngleRad;

  double currentExtensionDistance = 0.0;
  double desiredExtensionDistance = currentExtensionDistance;


  PIDController pivotPID; 

  SlewRateLimiter pivotRateLimiter;
  PIDController extensionPID;
  DutyCycleEncoder absPivEncoder;
  DutyCycleEncoder absTelescopeEncoder;

  
  public ArmControlSubsystem() {

    pivotPID = new PIDController(1.4, 0, 0);
    
    pivotPID.setTolerance(Units.degreesToRadians(2));


    extensionPID = new PIDController(0.0688, 0, 0);
    extensionPID.setTolerance(.2);
    pivotRateLimiter = new SlewRateLimiter(ArmConstants.maxPivotRateRadSec);

    if(ArmConstants.useAbsEncoderPiv){
      absPivEncoder = new DutyCycleEncoder(ArmConstants.DIOPortPiv);
      this.initialPivotEncoderOffset = -(absPivEncoder.getAbsolutePosition() + ArmConstants.pivotInitOffset) * ArmConstants.pivotAbsEncToRotation;
      this.currentPivotRotation = Units.rotationsToRadians(this.initialPivotEncoderOffset);
      this.currentPivotRotation = this.desiredPivotRotation;
    }

    
   
    
    setConfig(false);


    SmartDashboard.putBoolean("armCoastMode", isCoast);
    
  }

  public void setConfig(boolean isCoast){
    rightPivotController.configFactoryDefault();
    leftPivotController.configFactoryDefault();

    rightPivotController.follow(leftPivotController);

    rightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    leftPivotController.setInverted(ArmConstants.leftPivotInverted);

    // rightPivotController.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
    // leftPivotController.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);

    this.updateModes();

    


    rightPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftPivotController.setSelectedSensorPosition(0);
    rightPivotController.setSelectedSensorPosition(0);

    
    //extensionController.setIdleMode(IdleMode.kBrake); //ccheck if needed
    extensionController.setInverted(false);

    extensionEncoder.setPosition(0);


    extensionController.burnFlash();
    
  }

  @Override
  public void periodic() {      
      if(Robot.armModeSelector.getSelected() == ArmMotorMode.BRAKE){
        pivotPeriodic(); //maintains the desired pivot angle
        extensionPeriodic();
      }

      updateModes();

      // Getting Current And Desired Distances
      SmartDashboard.putNumber("currentTelescopeOutput", currentExtensionDistance);
      SmartDashboard.putNumber("CurrentPivotPoint", Units.radiansToDegrees(currentPivotRotation));

      if (CompConstants.debug) {
        SmartDashboard.putNumber("DesiredPivotPoint", Units.radiansToDegrees(desiredPivotRotation));
        SmartDashboard.putNumber("desiredTelescopeOutput", desiredExtensionDistance);

      //Encoder Positions
      
        SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));

        SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
        SmartDashboard.putNumber("extensionEncoderPos", extensionEncoder.getPosition()*ArmConstants.extensionRotationToInches);
      
        SmartDashboard.putBoolean("isCoast", isCoast);
      }

      
      
      desiredPivotRotation = Util.clamp(desiredPivotRotation, ArmConstants.minAngleRad, ArmConstants.maxAngleRad);
      currentPivotRotation = getCurrentPivotRotation(true);


      desiredExtensionDistance = Util.clamp(desiredExtensionDistance, ArmConstants.minExtensionIn, ArmConstants.maxExtensionIn);
      currentExtensionDistance = getCurrentExtensionIn();
  }

  private void pivotPeriodic(){    
    // desiredPivotRotation = Util.clamp(desiredPivotRotation, ArmConstants.minAngleRad, ArmConstants.maxAngleRad);
    

    //set currentRotation with encoders
    // currentPivotRotation = getCurrentPivotRotation(true);

    double pivotPIDOutput = pivotPID.calculate(currentPivotRotation, desiredPivotRotation);



    if(pivotPIDOutput > 0.6){
      pivotPIDOutput = 0.6;
    }else if(pivotPIDOutput < -0.6){
      pivotPIDOutput = -0.6;
    }

    if (CompConstants.debug) {SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);}
    
    if (CompConstants.rateLimitArm) {
      pivotPIDOutput = pivotRateLimiter.calculate(pivotPIDOutput);
    }

    leftPivotController.set(pivotPIDOutput);
    rightPivotController.set(pivotPIDOutput);
  }

  private void extensionPeriodic(){
      


    //desiredExtensionDistance = Util.clamp(desiredExtensionDistance, ArmConstants.minExtensionIn, ArmConstants.maxExtensionIn);
    // if(desiredPivotRotation <= ArmConstants.minAngleRad ){
    //   desiredExtensionDistance = ArmConstants.minExtensionIn;
    // }


    //currentExtensionDistance = getCurrentExtensionIn();

    double extensionPIDOutput = extensionPID.calculate(currentExtensionDistance, desiredExtensionDistance);

    SmartDashboard.putNumber("extensionPIDOutput", extensionPIDOutput);

    double difference = desiredExtensionDistance - currentExtensionDistance;

    if(extensionPIDOutput > .8){
      extensionPIDOutput = .7;
    }else if (extensionPIDOutput < -0.8){
      extensionPIDOutput = -.8;
    }

    double offset =  .12 * (difference > 0 ? 1 : -1);
    if (Math.abs(difference) > .14){
      extensionController.set(offset + extensionPIDOutput);
    }else{
      extensionController.set(0);
    }

   
  }



  public void setDesiredPivotRotation(double _desiredRotation){
    desiredPivotRotation = _desiredRotation;
  }

  //in inches
  public void setDesiredExtension(double _extension){
    desiredExtensionDistance = _extension;
  }

  //these functions assume the camera is on the front of the drivebase 
  public double getDistanceFromPivotToPose(double distanceFromCamera){
    return Math.sqrt(Math.pow(distanceFromCamera, 2) + Math.pow(ArmConstants.pivotPosInMetersY, 2));
  }
  /**
   * 
   * @param distanceFromCamera in meters
   * @return the desired angle in radians
   */


  public double getDesiredPivotAngle(double distanceFromCamera){
    return Math.atan(distanceFromCamera / ArmConstants.pivotPosInMetersY); //in radians
  }

  public boolean atAngleSetpoint(){
    //return Math.abs(desiredPivotRotation - currentPivotRotation) < Units.degreesToRadians(2);
    return true;
  }


  public boolean atTelescopeSetpoint(){
   // return Math.abs(desiredExtensionDistance - currentExtensionDistance) < 1;
    return true;
  }


  public Command waitUntilSpPivot(double sp){
    return new FunctionalCommand(()->setDesiredPivotRotation(sp), null, null, this::atAngleSetpoint, this);
  }


  public Command waitUntilSpTelescope(double sp){
  
    return new FunctionalCommand(()->setDesiredExtension(sp), null, null, this::atTelescopeSetpoint, this);
  }
  

  public double getCurrentPivotRotation(boolean inRadians){
    //double rotation = pivotEncoder.getAbsolutePosition() - ArmConstants.pivotInitOffset;
    double rotation;
    if (ArmConstants.useAbsEncoderPiv){
      rotation = (leftPivotController.getSelectedSensorPosition()) * ArmConstants.encoderResolution * ArmConstants.falconToFinalGear + this.initialPivotEncoderOffset;
    }else{
      rotation = (leftPivotController.getSelectedSensorPosition()) * ArmConstants.encoderResolution * ArmConstants.falconToFinalGear + Units.radiansToRotations(ArmConstants.zeroAngleRad);
    }
    
    if(inRadians){
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }


  //convert encoder rotations to distance inches
  public double getCurrentExtensionIn(){
    return extensionEncoder.getPosition()*ArmConstants.extensionRotationToInches* ArmConstants.gearRatioExtension;
  }
  
  public void changeDesiredPivotRotation(double i){
    this.desiredPivotRotation += i;
  }
  public void changeDesiredExtension(double i){
    this.desiredExtensionDistance += i;
  }
  /**
   * returns the pose of the center of the claw relative to the base of the robot
   * @return a translation2d object in meters and radians X is horizontal dist from the center of bot and Y is vertical distance from the center of the robot
   */
  public Translation2d getRelativeClawPose(){
    double r = Units.inchesToMeters(getCurrentExtensionIn()) + IntakeConstants.clawLengthMeters/2 + ArmConstants.pivotPosInMetersY;
    double theta = getCurrentPivotRotation(true);

    return new Translation2d((r*Math.cos(theta)),(r*Math.sin(theta))); 
  }

  void updateModes(){
    if(Robot.armModeSelector.getSelected() == ArmMotorMode.BRAKE){
      rightPivotController.setNeutralMode(NeutralMode.Brake);
      leftPivotController.setNeutralMode(NeutralMode.Brake);
      extensionController.setIdleMode(IdleMode.kBrake);
    }
    else if(Robot.armModeSelector.getSelected() == ArmMotorMode.COAST){
      rightPivotController.setNeutralMode(NeutralMode.Coast);
      leftPivotController.setNeutralMode(NeutralMode.Coast);
      extensionController.setIdleMode(IdleMode.kCoast);
    }
  }
  

  public void resetEncoders(){
    rightPivotController.setSelectedSensorPosition(0);
    leftPivotController.setSelectedSensorPosition(0);
    extensionEncoder.setPosition(0);

    
    if (ArmConstants.useAbsEncoderPiv){
      this.initialPivotEncoderOffset = -(absPivEncoder.getAbsolutePosition() + ArmConstants.pivotInitOffset) * ArmConstants.pivotAbsEncToRotation;
      this.currentPivotRotation = Units.rotationsToRadians(this.initialPivotEncoderOffset);
      this.currentPivotRotation = this.desiredPivotRotation;
    }else{
      this.currentPivotRotation = ArmConstants.zeroAngleRad;
      this.currentPivotRotation = this.desiredPivotRotation;
    }

  }

  public void moveToEnum(ArmSetting set) {
    double extension = ArmConstants.minExtensionIn;
    double rotation = ArmConstants.minAngleRad;
    switch (set) {
      case GNODE:
        break;
      case NEUTRAL:
        extension = ArmConstants.minExtensionIn;
        rotation = ArmConstants.minAngleRad;
        break;
      case NODE1:
        extension = ArmConstants.extensionLevelsIn[0];
        rotation = ArmConstants.angleLevelsDeg[0];
        break;
      case NODE2:
        extension = ArmConstants.extensionLevelsIn[1];
        rotation = ArmConstants.angleLevelsDeg[1];
        break;
      case NODE3:
        extension = ArmConstants.extensionLevelsIn[2];
        rotation = ArmConstants.angleLevelsDeg[2];
        break;
      case SUBSTATION:
        extension = ArmConstants.offSubstation[1];
        rotation = ArmConstants.offSubstation[2];
      default:
        extension = ArmConstants.minExtensionIn;
        rotation = ArmConstants.minAngleRad; 
        break;
    }

    setDesiredPivotRotation(rotation);
    setDesiredExtension(extension);
  }

  

}
