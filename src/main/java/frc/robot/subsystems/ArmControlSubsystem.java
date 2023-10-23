package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import javax.print.attribute.standard.PrinterMoreInfo;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CompConstants;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.Robot;

public class ArmControlSubsystem extends SubsystemBase {

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
  

  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.leftArmPivot);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.rightArmPivot);
  private final DutyCycleEncoder absPivEncoder = new DutyCycleEncoder(ArmConstants.DIOPortPiv);
  
  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.telescopicArmSpark, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();


  double initialPivotEncoderOffset;
  double currentPivotRotation;
  double desiredPivotRotation = ArmConstants.minAngleRad;

  double currentExtensionDistance = ArmConstants.zeroExtensionIn;
  double desiredExtensionDistance = ArmConstants.minExtensionIn;

  PIDController pivotPID; 

  SlewRateLimiter pivotRateLimiter;
  PIDController extensionPID;

  DutyCycleEncoder absTelescopeEncoder;

  boolean isInitilized = false;

  public SendableChooser<ArmMotorMode> chooser = new SendableChooser<>();


  public ArmControlSubsystem() {

    pivotPID = new PIDController(1.4, 0, 0);
    pivotPID.setTolerance(Units.degreesToRadians(0));

    extensionPID = new PIDController(0.19, 0, 0);
    extensionPID.setTolerance(.2);

    pivotRateLimiter = new SlewRateLimiter(ArmConstants.maxPivotRateRadSec);

    absPivEncoder.setConnectedFrequencyThreshold(975); //do not change this number pls or else 
   
    this.initialPivotEncoderOffset = -absPivEncoder.getAbsolutePosition() * ArmConstants.pivotAbsEncToRotation + ArmConstants.pivotInitOffsetRot;
   
    setConfig();

    SmartDashboard.putNumber("kG", ArmConstants.kG);
    
    chooser.addOption("Brake", ArmMotorMode.BRAKE);
    chooser.addOption("Coast", ArmMotorMode.COAST); 
    chooser.setDefaultOption("Brake", ArmMotorMode.BRAKE);

    SmartDashboard.putData("ArmMotorMode", chooser);
    
  }

  public void setConfig(){
    rightPivotController.configFactoryDefault();
    leftPivotController.configFactoryDefault();

    rightPivotController.follow(leftPivotController);

    rightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    leftPivotController.setInverted(ArmConstants.leftPivotInverted);

    this.updateModes();

    rightPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftPivotController.setSelectedSensorPosition(0);
    rightPivotController.setSelectedSensorPosition(0);
    
    extensionController.setInverted(false);

    extensionEncoder.setPosition(0);


    extensionController.burnFlash();
    
  }

  @Override
  public void periodic() { 
    updateCurrentState();     
      if(chooser.getSelected() == ArmMotorMode.BRAKE && isInitilized){
       pivotPeriodic(); //maintains the desired pivot angle
       extensionPeriodic();
      }else if(chooser.getSelected() == ArmMotorMode.COAST){
        desiredPivotRotation = currentPivotRotation;
        desiredExtensionDistance = currentExtensionDistance;
      }

      if(!isInitilized && -absPivEncoder.getAbsolutePosition() * ArmConstants.pivotAbsEncToRotation != 0 && absPivEncoder.isConnected()){
        //What does this line do?
        this.initialPivotEncoderOffset = -absPivEncoder.getAbsolutePosition() * ArmConstants.pivotAbsEncToRotation +.032;
        
        this.currentPivotRotation = Units.rotationsToRadians(this.initialPivotEncoderOffset);
        this.desiredPivotRotation = this.currentPivotRotation;
        
        isInitilized = true;
      }
      
      updateModes();
    

      // Getting Current And Desired Distances
      SmartDashboard.putNumber("CurrentExtension", this.getCurrentExtensionIn());
      SmartDashboard.putNumber("CurrentPivotDeg", Units.radiansToDegrees(currentPivotRotation));

      if (CompConstants.kDebugMode) {
        ArmConstants.kG =  SmartDashboard.getNumber("kG", ArmConstants.kG);

        SmartDashboard.putNumber("DesiredPivotDeg", Units.radiansToDegrees(desiredPivotRotation));
        SmartDashboard.putNumber("DesiredExtension", desiredExtensionDistance);

      //Encoder Positions
      
        SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));
        SmartDashboard.putNumber("AbsPivot", -absPivEncoder.getAbsolutePosition() * ArmConstants.pivotAbsEncToRotation);
        SmartDashboard.putNumber("InitialAbsPivot", this.initialPivotEncoderOffset);

        //this.initialPivotEncoderOffset = -absPivEncoder.getAbsolutePosition() * ArmConstants.pivotAbsEncToRotation;

        SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
        SmartDashboard.putNumber("extensionEncoderPos", extensionEncoder.getPosition()*ArmConstants.extensionRotationToInches);
      
        SmartDashboard.putNumber("pivotfreq", absPivEncoder.getFrequency());
      
        SmartDashboard.putBoolean("AtAnglePoint", this.atAngleSetpoint());
        SmartDashboard.putBoolean("AtExtensionPoint", this.atTelescopeSetpoint());
        SmartDashboard.putBoolean("IsInititilized", isInitilized);
        SmartDashboard.putBoolean("absreconnectioned", absPivEncoder.isConnected());

      }

      
      
      desiredPivotRotation = MathUtil.clamp(desiredPivotRotation, ArmConstants.minAngleRad, ArmConstants.maxAngleRad);
      currentPivotRotation = getCurrentPivotRotation(true);

      if (!CompConstants.ultraInstinct) {
        desiredExtensionDistance = MathUtil.clamp(desiredExtensionDistance, ArmConstants.minExtensionIn, ArmConstants.maxExtensionIn);
      }
      currentExtensionDistance = getCurrentExtensionIn();
  }

  private void pivotPeriodic(){    
    double pivotPIDOutput = pivotPID.calculate(currentPivotRotation, desiredPivotRotation);

    //Desaturating PID output
    if(pivotPIDOutput > 0.55){
      pivotPIDOutput = 0.55;
    }else if(pivotPIDOutput < -0.55){
      pivotPIDOutput = -0.55;
    }

    if (CompConstants.kDebugMode) {SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);}
    
    if (ArmConstants.kRateLimitArm) {
      pivotPIDOutput = pivotRateLimiter.calculate(pivotPIDOutput);
    }
    if(ArmConstants.useFeedForward){
      pivotPIDOutput += ArmConstants.kG*Math.cos(getCurrentPivotRotation(true)-(Math.PI/2));
    }

    leftPivotController.set(pivotPIDOutput);
    rightPivotController.set(pivotPIDOutput);
  }

  private void extensionPeriodic(){

    double extensionPIDOutput = extensionPID.calculate(currentExtensionDistance, desiredExtensionDistance);
    SmartDashboard.putNumber("extensionPIDOutput", extensionPIDOutput);
    double difference = desiredExtensionDistance - currentExtensionDistance;

    //Desaturating PID output
    if(extensionPIDOutput > .5){
      extensionPIDOutput = .5;
    }else if (extensionPIDOutput < -0.5){
      extensionPIDOutput = -.5;
    }

    //This is for handling the friction in the extension
    double offset = .005 * (difference > 0 ? 1 : -1);
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

  public boolean atAngleSetpoint(){
    return Math.abs(desiredPivotRotation - currentPivotRotation) < Units.degreesToRadians(4);
    //return true;
  }

  public boolean atTelescopeSetpoint(){
    return Math.abs(desiredExtensionDistance - currentExtensionDistance) < 0.5;
    //return true;
  }

  public Command waitUntilSpPivot(double sp){
    return new FunctionalCommand(()->setDesiredPivotRotation(sp), ()->new PrintCommand("getName()"), (Boolean bool)->new PrintCommand("Finished Pivot"), this::atAngleSetpoint, this);
  }


  public Command waitUntilSpTelescope(double sp){
    return new FunctionalCommand(()->setDesiredExtension(sp), ()->new PrintCommand("finished"), (Boolean bool)-> new PrintCommand("getName()"), this::atTelescopeSetpoint, this);
  }
  

  public double getCurrentPivotRotation(boolean inRadians){
    double rotation;

    rotation = (leftPivotController.getSelectedSensorPosition()) * ArmConstants.encoderResolution * ArmConstants.falconToFinalGear + this.initialPivotEncoderOffset;
    
    if(inRadians){
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }


  //convert encoder rotations to distance inches
  public double getCurrentExtensionIn(){
    return extensionEncoder.getPosition()*8.333/8.113;
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
  // public Translation2d getRelativeClawPose(){
  //   double r = Units.inchesToMeters(getCurrentExtensionIn()) + IntakeConstants.clawLengthMeters/2 + ArmConstants.pivotPosInMetersY;
  //   double theta = getCurrentPivotRotation(true);

  //   return new Translation2d((r*Math.cos(theta)),(r*Math.sin(theta))); 
  // }

  void updateModes(){
    SmartDashboard.updateValues();
    if(chooser.getSelected() == ArmMotorMode.BRAKE){
      rightPivotController.setNeutralMode(NeutralMode.Brake);
      leftPivotController.setNeutralMode(NeutralMode.Brake);
      extensionController.setIdleMode(IdleMode.kBrake);
    }
    else if(chooser.getSelected() == ArmMotorMode.COAST){
      rightPivotController.setNeutralMode(NeutralMode.Coast);
      leftPivotController.setNeutralMode(NeutralMode.Coast);
      extensionController.setIdleMode(IdleMode.kCoast);
    }
  }

  void updateCurrentState(){
    for (ArmState arm:ArmConstants.ArmState.values()){
      if (Math.abs(this.getCurrentPivotRotation(true)-arm.pivotAngleRad)<ArmConstants.armStatePivDeadzoneRad && Math.abs(this.getCurrentExtensionIn()-arm.extentionDistIn)<ArmConstants.armStateExtDeadzoneIn){
        ArmConstants.curArmState = arm;
      }
    }
  }
  

  public void resetEncoders(){
    rightPivotController.setSelectedSensorPosition(0);
    leftPivotController.setSelectedSensorPosition(0);
    extensionEncoder.setPosition(0);

    this.initialPivotEncoderOffset = -(absPivEncoder.getAbsolutePosition() + ArmConstants.pivotInitOffsetRot) * ArmConstants.pivotAbsEncToRotation;
    this.currentPivotRotation = Units.rotationsToRadians(this.initialPivotEncoderOffset);
    this.currentPivotRotation = this.desiredPivotRotation;
  }
}