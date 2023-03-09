package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
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
  
  
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.rightArmPivot);
  //private final AnalogEncoder pivotEncoder = new AnalogEncoder(ArmConstants.armPivotEncoderPort);

  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();
  

  double currentPivotRotation = ArmConstants.minAngleRad;
  double desiredPivotRotation = currentPivotRotation;

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
    
    

    rightPivotController.setInverted(true);
    SmartDashboard.putNumber("PivotkP", 2);
  }

  @Override
  public void periodic() {
      pivotPeriodic(); //maintains the desired pivot angle
      extensionPeriodic(); //maintains the desired extension length
  }

  private void pivotPeriodic(){

    pivotPID = new PIDController(SmartDashboard.getNumber("PivotkP", 0), 0, 0);

    desiredPivotRotation = Util.clamp(desiredPivotRotation, ArmConstants.minAngleRad, ArmConstants.maxAngleRad);

    //set currentRotation with encoders
    currentPivotRotation = getCurrentPivotRotation(true);

    double pivotFeedforwardOutput = pivotFeedforward.calculate(desiredPivotRotation, 1, 1); //arbitrary
    double pivotPIDOutput = pivotPID.calculate(currentPivotRotation, desiredPivotRotation);

    SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);
    SmartDashboard.putNumber("CurrentPivotPoint", currentPivotRotation);
    SmartDashboard.putNumber("DesiredPivotPoint", desiredPivotRotation);
    SmartDashboard.putNumber("LeftSensor", leftPivotController.getSelectedSensorPosition() / 2048 * ArmConstants.relEncoderToInitialGear);
    SmartDashboard.putNumber("RightSensor", rightPivotController.getSelectedSensorPosition() / 2048 * ArmConstants.relEncoderToInitialGear);

     
    
    
    //TODO we dont know which one is inverted yet
    leftPivotController.set(pivotPIDOutput + (pivotFeedforwardOutput / RobotController.getBatteryVoltage())); 
    rightPivotController.set(pivotPIDOutput + (pivotFeedforwardOutput / RobotController.getBatteryVoltage())); 
  }

  private void extensionPeriodic(){


    desiredExtensionDistance = Util.clamp(desiredExtensionDistance, ArmConstants.minExtensionIn, ArmConstants.maxExtensionIn);
    if(desiredPivotRotation <= ArmConstants.minPivotForExtensionRad){
      desiredExtensionDistance = ArmConstants.minExtensionIn;
    }



    currentExtensionDistance = getCurrentExtensionIn();

    double extensionPIDOutput = extensionPID.calculate(currentExtensionDistance, desiredExtensionDistance);

    extensionController.set(extensionPIDOutput);
  }

  public void setConfig(){
    rightPivotController.follow(leftPivotController);
    rightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    leftPivotController.setInverted(ArmConstants.leftPivotInverted);
    rightPivotController.setNeutralMode(NeutralMode.Brake);
    leftPivotController.setNeutralMode(NeutralMode.Brake);
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
    return extensionPID.atSetpoint();
  }

  public Command waitUntilSpPivot(double sp){
    return new FunctionalCommand(()->setDesiredPivotRotation(sp), null, null, this::atAngleSetpoint, this);
  }

  public Command waitUntilSpTelescope(double sp){
    return new FunctionalCommand(()->setDesiredExtension(sp), null, null, this::atTelescopeSetpoint, this);
  }
  
  public double getCurrentPivotRotation(boolean inRadians){
    //double rotation = pivotEncoder.getAbsolutePosition() - ArmConstants.pivotInitOffset;
    double rotation = rightPivotController.getSelectedSensorPosition() / 2048 / 240;
    if(inRadians)
      return rotation * Math.PI * 2 % (Math.PI * 2);
    return rotation;
  }

  //convert encoder rotations to distance inches
  public double getCurrentExtensionIn(){
    return extensionEncoder.getPosition()*ArmConstants.extensionEncoderToLength + ArmConstants.minExtensionIn;
  }
  
  public void changeDesiredPivotRotation(double i){
    this.desiredPivotRotation += i;
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