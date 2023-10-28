package frc.robot.subsystems;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CompConstants;


public class ArmControlSubsystem extends SubsystemBase {

  public enum ArmMotorMode {
    COAST,
    BRAKE,
    OFF
  }

  public enum MoveArmConfig {
    SIMULTANEOUS,
    SEQ_EXTEND_THEN_PIVOT,
    SEQ_PIVOT_THEN_EXTEND
  }
  
  
  private final WPI_TalonFX leftPivotController = new WPI_TalonFX(ArmConstants.LEFT_PIV_MOTOR_ID);
  private final WPI_TalonFX rightPivotController = new WPI_TalonFX(ArmConstants.RIGHT_PIV_MOTOR_ID);
  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.EXT_SPARK_ID, MotorType.kBrushless);

  private final DutyCycleEncoder absPivEncoder = new DutyCycleEncoder(ArmConstants.DIOPORTPIV);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();

  private final SlewRateLimiter pivotRateLimiter;
  private final PIDController extensionPID;
  private final PIDController pivotPID;

  // This is called Ultra Instinct because setting this boolean to true removes the clamps
  // so that if the belt skips, the operator can still run manually without the bad offsets
  private boolean ultraInstinct = false;

  private double pivotRelEncoderOffsetRot;
  private double currentPivotRotation;
  private double desiredPivotRotation = ArmConstants.MIN_PIV_ANGLE_RAD;

  private double currentExtensionDistance = ArmConstants.ZERO_EXTENSION_IN;
  private double desiredExtensionDistance = ArmConstants.MIN_EXT_LEN_IN;



  private boolean isInitialized = false;

  private final SendableChooser<ArmMotorMode> chooser = new SendableChooser<>();


  public ArmControlSubsystem() {

    pivotPID = new PIDController(1.4, 0, 0);
    pivotPID.setTolerance(Units.degreesToRadians(0));

    extensionPID = new PIDController(0.19, 0, 0);
    extensionPID.setTolerance(.2);

    pivotRateLimiter = new SlewRateLimiter(ArmConstants.MAX_PIV_RATE_RAD_SEC);

    absPivEncoder.setConnectedFrequencyThreshold(975); //do not change this number pls or else 
   
    this.pivotRelEncoderOffsetRot = -absPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION + ArmConstants.PIV_INIT_OFFSET_ROT;
   
    setConfig();

    SmartDashboard.putNumber("kG", ArmConstants.KG);
    
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
    leftPivotController.setInverted(ArmConstants.LEFT_PIV_MOTOR_INVERTED);

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
    if(chooser.getSelected() == ArmMotorMode.BRAKE && isInitialized){
      pivotPeriodic();
      extensionPeriodic();
    }else if(chooser.getSelected() == ArmMotorMode.COAST){
      desiredPivotRotation = currentPivotRotation;
      desiredExtensionDistance = currentExtensionDistance;
    }

    if(!isInitialized && -absPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION != 0 && absPivEncoder.isConnected()){
      resetEncoders();   
      isInitialized = true;
    }
    
    updateModes();
  

    // Getting Current And Desired Distances
    SmartDashboard.putNumber("CurrentExtension", this.getCurrentExtensionIn());
    SmartDashboard.putNumber("CurrentPivotDeg", Units.radiansToDegrees(currentPivotRotation));

    if (CompConstants.DEBUG_MODE) {
      //Encoder Positions
      SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));
      SmartDashboard.putNumber("InitialAbsPivot", this.pivotRelEncoderOffsetRot);

      SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
      SmartDashboard.putNumber("extensionEncoderPos", extensionEncoder.getPosition()*ArmConstants.EXTENSION_ROTATION_TO_INCHES);
    
      //Abs Encoder Debugging
      SmartDashboard.putNumber("pivotfreq", absPivEncoder.getFrequency());
      SmartDashboard.putBoolean("IsInititilized", isInitialized);
      SmartDashboard.putBoolean("absreconnectioned", absPivEncoder.isConnected());
      SmartDashboard.putNumber("AbsPivot", -absPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION);

      //Setpoint Debugging
      SmartDashboard.putBoolean("AtAnglePoint", this.atAngleSetpoint());
      SmartDashboard.putBoolean("AtExtensionPoint", this.atTelescopeSetpoint());
      SmartDashboard.putNumber("DesiredPivotDeg", Units.radiansToDegrees(desiredPivotRotation));
      SmartDashboard.putNumber("DesiredExtension", desiredExtensionDistance);


    }
    
    desiredPivotRotation = MathUtil.clamp(desiredPivotRotation, ArmConstants.MIN_PIV_ANGLE_RAD, ArmConstants.MAX_PIV_ANGLE_RAD);
    currentPivotRotation = getCurrentPivotRotation(true);

    if (!ultraInstinct) {
      desiredExtensionDistance = MathUtil.clamp(desiredExtensionDistance, ArmConstants.MIN_EXT_LEN_IN, ArmConstants.MAX_EXT_LEN_IN);
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

    if (CompConstants.DEBUG_MODE) {SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);}
    
    if (ArmConstants.RATE_LIMIT_ARM) {
      pivotPIDOutput = pivotRateLimiter.calculate(pivotPIDOutput);
    }
    if(ArmConstants.ENABLE_FEEDFORWARD){
      pivotPIDOutput += ArmConstants.KG*Math.cos(getCurrentPivotRotation(true)-(Math.PI/2));
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


  public void setDesiredPivotRot(double desiredRotation){
    desiredPivotRotation = desiredRotation;
  }


  //in inches
  public void setDesiredExtension(double extension){
    desiredExtensionDistance = extension;
  }


  public boolean atAngleSetpoint(){
    return Math.abs(desiredPivotRotation - currentPivotRotation) < Units.degreesToRadians(4);
  }


  public boolean atTelescopeSetpoint(){
    return Math.abs(desiredExtensionDistance - currentExtensionDistance) < 0.5;
  }


  public Command waitUntilSpPivot(double sp){
    return new FunctionalCommand(()->setDesiredPivotRot(sp), ()->new PrintCommand("getName()"), (Boolean bool)->new PrintCommand("Finished Pivot"), this::atAngleSetpoint, this);
  }


  public Command waitUntilSpTelescope(double sp){
    return new FunctionalCommand(()->setDesiredExtension(sp), ()->new PrintCommand("finished"), (Boolean bool)-> new PrintCommand("getName()"), this::atTelescopeSetpoint, this);
  }
  

  public double getCurrentPivotRotation(boolean inRadians){
    double rotation;

    rotation = (leftPivotController.getSelectedSensorPosition()) * ArmConstants.ENCODER_RES * ArmConstants.PIV_MOTOR_TO_GEAR_ROT + this.pivotRelEncoderOffsetRot;
    
    if(inRadians){
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }


  //convert encoder rotations to distance inches
  public double getCurrentExtensionIn(){
    return extensionEncoder.getPosition()*ArmConstants.GEAR_RATIO_EXTENSION_IN;
  }

  
  public void changeDesiredPivotRotation(double i){
    this.desiredPivotRotation += i;
  }


  public void changeDesiredExtension(double i){
    this.desiredExtensionDistance += i;
  }

  public boolean getUltraInstinct(){
    return ultraInstinct;
  }
  public void setUltraInstinct(boolean toggle){
    ultraInstinct = toggle;
  }


  private double getPivotAbsEncoderAngleRot(){
    return -absPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION + this.pivotRelEncoderOffsetRot;
  }

  private void updateModes(){
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

  private void resetEncoders(){
    rightPivotController.setSelectedSensorPosition(0);
    leftPivotController.setSelectedSensorPosition(0);
    extensionEncoder.setPosition(0);

    this.pivotRelEncoderOffsetRot = getPivotAbsEncoderAngleRot();
    this.currentPivotRotation = Units.rotationsToRadians(this.pivotRelEncoderOffsetRot);
    this.currentPivotRotation = this.desiredPivotRotation;
  }
}