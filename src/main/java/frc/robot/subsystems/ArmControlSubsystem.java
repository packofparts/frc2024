package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util;


public class ArmControlSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.rightArmPivot);
  private final AnalogEncoder pivotEncoder = new AnalogEncoder(ArmConstants.armPivotEncoderPort);

  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();
  

  double currentPivotRotation = ArmConstants.minAngle;
  double desiredPivotRotation = currentPivotRotation;

  double currentExtensionDistance = ArmConstants.minExtension;
  double desiredExtensionDistance = currentExtensionDistance;

  //TODO moves these to ArmConstants
  PIDController pivotPID; //TODO calculate gains to actually change the angle
  ArmFeedforward pivotFeedforward; //TODO calculate gains to beat the force of gravity 

  PIDController extensionPID;
  
  //TODO make the constructor more useful and modular by passing in most values from ArmConstants
  public ArmControlSubsystem() {
    pivotPID = new PIDController(0.1, 0, 0); //TODO calculate gains to actually change the angle
    pivotFeedforward = new ArmFeedforward(0, 0, 0, 0); //TODO calculate gains to beat the force of gravity 

    extensionPID = new PIDController(0.1, 0, 0);
  }

  @Override
  public void periodic() {
      pivotPeriodic(); //maintains the desired pivot angle
      extensionPeriodic(); //maintains the desired extension length
  }

  private void pivotPeriodic(){
    desiredPivotRotation = Util.clamp(desiredPivotRotation, ArmConstants.minAngle, ArmConstants.maxAngle);

    //set currentRotation with encoders
    currentPivotRotation = getCurrentPivotRotation(true);

    double pivotFeedforwardOutput = pivotFeedforward.calculate(desiredPivotRotation, 1, 1); //arbitrary
    double pivotPIDOutput = pivotPID.calculate(currentPivotRotation, desiredPivotRotation); 
    
    
    //TODO we dont know which one is inverted yet
    leftPivotController.set(pivotPIDOutput + (pivotFeedforwardOutput / RobotController.getBatteryVoltage())); 
    rightPivotController.set(-pivotPIDOutput + (-pivotFeedforwardOutput / RobotController.getBatteryVoltage()));
  }

  private void extensionPeriodic(){
    desiredExtensionDistance = Util.clamp(desiredExtensionDistance, ArmConstants.minExtension, ArmConstants.maxExtension);

    currentExtensionDistance = getCurrentExtension();

    double extensionPIDOutput = extensionPID.calculate(currentExtensionDistance, desiredExtensionDistance);

    extensionController.set(extensionPIDOutput);
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

  public double getDesiredPivotAngle(double distanceFromCamera){
    return Math.atan(distanceFromCamera / ArmConstants.pivotPosInMetersY); //in radians
  }


  public double getCurrentPivotRotation(boolean inRadians){
    double rotation = pivotEncoder.getAbsolutePosition() - ArmConstants.pivotInitOffset;
    if(inRadians)
      return rotation * Math.PI * 2;
    return rotation;
  }

  //convert encoder rotations to distance inches
  public double getCurrentExtension(){
    return extensionEncoder.getPosition()*ArmConstants.extensionEncoderToLength + ArmConstants.minExtension;
  }
  
  public void changeDesiredPivotRotation(double i){
    this.desiredPivotRotation += i;
  }





}