
package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PnumaticConstants;
import frc.robot.Util;


public class ArmControlSubsystem extends SubsystemBase {
  
  public static enum ArmSetting {
    NODE3,
    NODE2,
    NODE1,
    GNODE,
    NEUTRAL,
  }
  
  int ii = 0;
  boolean isCoast = false;
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.rightArmPivot);
  //private final AnalogEncoder pivotEncoder = new AnalogEncoder(ArmConstants.armPivotEncoderPort);

  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();
  

  double currentPivotRotation = Units.degreesToRadians(ArmConstants.zeroAngleRad);
  double desiredPivotRotation = ArmConstants.minAngleRad;

  double currentExtensionDistance = ArmConstants.minExtensionIn;
  double desiredExtensionDistance = currentExtensionDistance;
  //TODO moves these to ArmConstants
  PIDController pivotPID; //TODO calculate gains to actually change the angle
  ArmFeedforward pivotFeedforward; //TODO calculate gains to beat the force of gravity 

  PIDController extensionPID;
  
  //TODO make the constructor more useful and modular by passing in most values from ArmConstants
  public ArmControlSubsystem() {
    //pivotPID = new PIDController(2, 0, 0); //TODO calculate gains to actually change the angle
    pivotPID = new PIDController(0.05, 0, 0);
    pivotFeedforward = new ArmFeedforward(0, 0, 0, 0); //TODO calculate gains to beat the force of gravity 

    extensionPID = new PIDController(0.05, 0, 0);
    
   
    
    setConfig(isCoast);
    SmartDashboard.putNumber("PivotkP", 2);
    SmartDashboard.putBoolean("armCoastMode", isCoast);
    

    desiredPivotRotation = getCurrentPivotRotation(true);
  }

  public void setConfig(boolean isCoast){
    rightPivotController.configFactoryDefault();
    leftPivotController.configFactoryDefault();

    rightPivotController.follow(leftPivotController);

    rightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    leftPivotController.setInverted(ArmConstants.leftPivotInverted);

    rightPivotController.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);
    leftPivotController.setNeutralMode(isCoast ? NeutralMode.Coast : NeutralMode.Brake);

    rightPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftPivotController.setSelectedSensorPosition(Units.radiansToRotations(ArmConstants.zeroAngleRad)*(1.0/ArmConstants.encoderResolution)*(1.0/ArmConstants.falconToFinalGear));
    rightPivotController.setSelectedSensorPosition(Units.radiansToRotations(ArmConstants.zeroAngleRad)*(1.0/ArmConstants.encoderResolution)*(1.0/ArmConstants.falconToFinalGear));

    extensionController.setIdleMode(IdleMode.kBrake);
    extensionController.setInverted(true);
    extensionEncoder.setPositionConversionFactor(ArmConstants.extensionEncoderToInches);
    extensionEncoder.setPosition(ArmConstants.minExtensionIn);
    extensionController.burnFlash();
    
  }

  @Override
  public void periodic() {
      // boolean br = SmartDashboard.getBoolean("armCoastMode", false);
      // if (br != isCoast){
      //   setConfig(br);
      // }

      //pivotPeriodic(); //maintains the desired pivot angle
      //extensionPeriodic(); //maintains the desired extension length
      SmartDashboard.putNumber("currentTelescopeOutput", currentExtensionDistance);
      SmartDashboard.putNumber("desiredTelescopeOutput", desiredExtensionDistance);

      SmartDashboard.putNumber("CurrentPivotPoint", Units.radiansToDegrees(currentPivotRotation));
      SmartDashboard.putNumber("DesiredPivotPoint", Units.radiansToDegrees(desiredPivotRotation));
      SmartDashboard.putNumber("LeftSensor", (leftPivotController.getSelectedSensorPosition()) / 2048 * ArmConstants.falconToFinalGear*360+60);
      SmartDashboard.putNumber("RightSensor", (rightPivotController.getSelectedSensorPosition()) / 2048 * ArmConstants.falconToFinalGear*360+60);
  
  
      SmartDashboard.putNumber("LeftPivotAbsPos",leftPivotController.getSensorCollection().getIntegratedSensorPosition()/2048*360);
      SmartDashboard.putNumber("LeftPivotIntegratedRelPos",leftPivotController.getSensorCollection().getIntegratedSensorPosition()/2048*360);


      SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
      SmartDashboard.putNumber("extensionEncoderPos", extensionEncoder.getPosition());
  }

  private void pivotPeriodic(){

    pivotPID = new PIDController(SmartDashboard.getNumber("PivotkP", 0), 0, 0);

    desiredPivotRotation = Util.clamp(desiredPivotRotation, ArmConstants.minAngleRad, ArmConstants.maxAngleRad);
    
    //set currentRotation with encoders
    currentPivotRotation = getCurrentPivotRotation(true);

    double pivotFeedforwardOutput = pivotFeedforward.calculate(desiredPivotRotation, 1, 1); //arbitrary
    double pivotPIDOutput = pivotPID.calculate(currentPivotRotation, desiredPivotRotation);

    SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);
    SmartDashboard.putNumber("CurrentPivotPoint", Units.radiansToDegrees(currentPivotRotation));
    SmartDashboard.putNumber("DesiredPivotPoint", Units.radiansToDegrees(desiredPivotRotation));



    SmartDashboard.putNumber("LeftPivotAbsPos",leftPivotController.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("LeftPivotIntegratedRelPos",leftPivotController.getSensorCollection().getIntegratedSensorPosition());
    
    //-0.095237523317337
    //0.142857700586319
    //TODO we dont know which one is inverted yet
    leftPivotController.set(pivotPIDOutput);
    rightPivotController.set(pivotPIDOutput); 
  }

  private void extensionPeriodic(){


    //desiredExtensionDistance = Util.clamp(desiredExtensionDistance, ArmConstants.minExtensionIn, ArmConstants.maxExtensionIn);
    // if(desiredPivotRotation <= ArmConstants.minPivotForExtensionRad){
    //   desiredExtensionDistance = ArmConstants.minExtensionIn;
    // }


    currentExtensionDistance = getCurrentExtensionIn();

    double extensionPIDOutput = extensionPID.calculate(currentExtensionDistance, desiredExtensionDistance);
    SmartDashboard.putNumber("extensionPIDOutput", extensionPIDOutput);
    
    extensionController.set(extensionPIDOutput);

  }



  public void setDesiredPivotRotation(double _desiredRotation){
    desiredPivotRotation = _desiredRotation;
    //rotates the rotater 
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
    return pivotPID.atSetpoint();
  }


  public boolean atTelescopeSetpoint(){
    //return extensionPID.atSetpoint();
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
    double rotation = (leftPivotController.getSelectedSensorPosition()) *ArmConstants.encoderResolution * ArmConstants.falconToFinalGear;

    rotation = Math.IEEEremainder(rotation, 1.0);
    
    if(inRadians){
      return rotation * 2*Math.PI;
    }
    return rotation;
  }


  //convert encoder rotations to distance inches
  public double getCurrentExtensionIn(){
    return extensionEncoder.getPosition();
  }
  
  public void changeDesiredPivotRotation(double i){
    this.desiredPivotRotation += i;
  }
  public void changeDesiredExtension(double i){
    this.desiredExtensionDistance += i;
    extensionController.set(i);
  }
  /**
   * returns the pose of the center of the claw relative to the base of the robot
   * @return a translation2d object in meters and radians X is horizontal dist from the center of bot and Y is vertical distance from the center of the robot
   */
  public Translation2d getRelativeClawPose(){
    double r = Units.inchesToMeters(getCurrentExtensionIn()) + PnumaticConstants.clawLengthMeters/2 + ArmConstants.pivotPosInMetersY;
    double theta = getCurrentPivotRotation(true);

    return new Translation2d((r*Math.cos(theta)),(r*Math.sin(theta))); 
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
      default:
        extension = ArmConstants.minExtensionIn;
        rotation = ArmConstants.minAngleRad; 
        break;
    }

    setDesiredPivotRotation(rotation);
    setDesiredExtension(extension);
  }
}
