package frc.robot.subsystems;

import org.opencv.ml.ANN_MLP;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

public class ArmControlSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(Constants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(Constants.rightArmPivot);
  private final AnalogEncoder pivotEncoder = new AnalogEncoder(Constants.armPivotEncoderPort);

  private final CANSparkMax extensionController = new CANSparkMax(Constants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();
  

  double currentPivotRotation = Constants.minAngle;
  double desiredPivotRotation = currentPivotRotation;

  double currentExtensionDistance = Constants.minExtension;
  double desiredExtensionDistance = currentExtensionDistance;

  //TODO moves these to constants
  PIDController pivotPID; //TODO calculate gains to actually change the angle
  ArmFeedforward pivotFeedforward; //TODO calculate gains to beat the force of gravity 

  PIDController extensionPID;
  
  //TODO make the constructor more useful and modular by passing in most values from constants
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
    desiredPivotRotation = Util.clamp(desiredPivotRotation, Constants.minAngle, Constants.maxAngle);

    //set currentRotation with encoders
    currentPivotRotation = getCurrentPivotRotation(true);

    double pivotFeedforwardOutput = pivotFeedforward.calculate(desiredPivotRotation, 1, 1); //arbitrary
    double pivotPIDOutput = pivotPID.calculate(currentPivotRotation, desiredPivotRotation); 
    
    
    //TODO we dont know which one is inverted yet
    leftPivotController.set(pivotPIDOutput + (pivotFeedforwardOutput / RobotController.getBatteryVoltage())); 
    rightPivotController.set(-pivotPIDOutput + (-pivotFeedforwardOutput / RobotController.getBatteryVoltage()));
  }

  private void extensionPeriodic(){
    desiredExtensionDistance = Util.clamp(desiredExtensionDistance, Constants.minExtension, Constants.maxExtension);

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
    return Math.sqrt(Math.pow(distanceFromCamera, 2) + Math.pow(Constants.pivotPosInMetersY, 2));
  }

  public double getDesiredPivotAngle(double distanceFromCamera){
    return Math.atan(distanceFromCamera / Constants.pivotPosInMetersY); //in radians
  }


  public double getCurrentPivotRotation(boolean inRadians){
    double rotation = pivotEncoder.getAbsolutePosition() - Constants.pivotInitOffset;
    if(inRadians)
      return rotation * Math.PI * 2;
    return rotation;
  }

  //convert encoder rotations to distance inches
  public double getCurrentExtension(){
    return extensionEncoder.getPosition()*Constants.extensionEncoderToLength + Constants.minExtension;
  }
  
  public void changeDesiredPivotRotation(double i){
    this.desiredPivotRotation += i;
  }





}