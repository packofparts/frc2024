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
  
  
  private final WPI_TalonFX _leftPivotController = new WPI_TalonFX(ArmConstants.kleftArmPivotID);
  private final WPI_TalonFX _rightPivotController = new WPI_TalonFX(ArmConstants.kRightArmPivotID);
  private final CANSparkMax extensionController = new CANSparkMax(ArmConstants.kTelescopicArmSparkID, MotorType.kBrushless);

  private final DutyCycleEncoder _absPivEncoder = new DutyCycleEncoder(ArmConstants.kDIOPortPiv);
  private final RelativeEncoder extensionEncoder = extensionController.getEncoder();

  private final SlewRateLimiter _pivotRateLimiter;
  private final PIDController _extensionPID;
  private final PIDController _pivotPID;

  public static boolean ultraInstinct = false;

  private double _pivotRelEncoderOffsetRot;
  private double _currentPivotRotation;
  private double _desiredPivotRotation = ArmConstants.kMinAngleRad;

  private double _currentExtensionDistance = ArmConstants.kZeroExtensionIn;
  private double _desiredExtensionDistance = ArmConstants.kMinExtensionIn;



  boolean isInitilized = false;

  public SendableChooser<ArmMotorMode> chooser = new SendableChooser<>();


  public ArmControlSubsystem() {

    _pivotPID = new PIDController(1.4, 0, 0);
    _pivotPID.setTolerance(Units.degreesToRadians(0));

    _extensionPID = new PIDController(0.19, 0, 0);
    _extensionPID.setTolerance(.2);

    _pivotRateLimiter = new SlewRateLimiter(ArmConstants.kMaxPivotRateRadSec);

    _absPivEncoder.setConnectedFrequencyThreshold(975); //do not change this number pls or else 
   
    this._pivotRelEncoderOffsetRot = -_absPivEncoder.getAbsolutePosition() * ArmConstants.kPivotAbsEncToRotation + ArmConstants.kPivotInitOffsetRot;
   
    setConfig();

    SmartDashboard.putNumber("kG", ArmConstants.kG);
    
    chooser.addOption("Brake", ArmMotorMode.BRAKE);
    chooser.addOption("Coast", ArmMotorMode.COAST); 
    chooser.setDefaultOption("Brake", ArmMotorMode.BRAKE);

    SmartDashboard.putData("ArmMotorMode", chooser);
    
  }

  public void setConfig(){
    _rightPivotController.configFactoryDefault();
    _leftPivotController.configFactoryDefault();

    _rightPivotController.follow(_leftPivotController);

    _rightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    _leftPivotController.setInverted(ArmConstants.kLeftPivotInverted);

    this.updateModes();

    _rightPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    _leftPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    _leftPivotController.setSelectedSensorPosition(0);
    _rightPivotController.setSelectedSensorPosition(0);
    
    extensionController.setInverted(false);

    extensionEncoder.setPosition(0);


    extensionController.burnFlash();
    
  }

  @Override
  public void periodic() { 
    if(chooser.getSelected() == ArmMotorMode.BRAKE && isInitilized){
      pivotPeriodic();
      extensionPeriodic();
    }else if(chooser.getSelected() == ArmMotorMode.COAST){
      _desiredPivotRotation = _currentPivotRotation;
      _desiredExtensionDistance = _currentExtensionDistance;
    }

    if(!isInitilized && -_absPivEncoder.getAbsolutePosition() * ArmConstants.kPivotAbsEncToRotation != 0 && _absPivEncoder.isConnected()){
      resetEncoders();   
      isInitilized = true;
    }
    
    updateModes();
  

    // Getting Current And Desired Distances
    SmartDashboard.putNumber("CurrentExtension", this.getCurrentExtensionIn());
    SmartDashboard.putNumber("CurrentPivotDeg", Units.radiansToDegrees(_currentPivotRotation));

    if (CompConstants.kDebugMode) {
      //Encoder Positions
      SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));
      SmartDashboard.putNumber("InitialAbsPivot", this._pivotRelEncoderOffsetRot);

      SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
      SmartDashboard.putNumber("extensionEncoderPos", extensionEncoder.getPosition()*ArmConstants.kExtensionRotationToInches);
    
      //Abs Encoder Debugging
      SmartDashboard.putNumber("pivotfreq", _absPivEncoder.getFrequency());
      SmartDashboard.putBoolean("IsInititilized", isInitilized);
      SmartDashboard.putBoolean("absreconnectioned", _absPivEncoder.isConnected());
      SmartDashboard.putNumber("AbsPivot", -_absPivEncoder.getAbsolutePosition() * ArmConstants.kPivotAbsEncToRotation);

      //Setpoint Debugging
      SmartDashboard.putBoolean("AtAnglePoint", this.atAngleSetpoint());
      SmartDashboard.putBoolean("AtExtensionPoint", this.atTelescopeSetpoint());
      SmartDashboard.putNumber("DesiredPivotDeg", Units.radiansToDegrees(_desiredPivotRotation));
      SmartDashboard.putNumber("DesiredExtension", _desiredExtensionDistance);


    }
    
    _desiredPivotRotation = MathUtil.clamp(_desiredPivotRotation, ArmConstants.kMinAngleRad, ArmConstants.kMaxAngleRad);
    _currentPivotRotation = getCurrentPivotRotation(true);

    if (!ultraInstinct) {
      _desiredExtensionDistance = MathUtil.clamp(_desiredExtensionDistance, ArmConstants.kMinExtensionIn, ArmConstants.kMaxExtensionIn);
    }
    _currentExtensionDistance = getCurrentExtensionIn();
  }

  private void pivotPeriodic(){    
    double pivotPIDOutput = _pivotPID.calculate(_currentPivotRotation, _desiredPivotRotation);

    //Desaturating PID output
    if(pivotPIDOutput > 0.55){
      pivotPIDOutput = 0.55;
    }else if(pivotPIDOutput < -0.55){
      pivotPIDOutput = -0.55;
    }

    if (CompConstants.kDebugMode) {SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);}
    
    if (ArmConstants.kRateLimitArm) {
      pivotPIDOutput = _pivotRateLimiter.calculate(pivotPIDOutput);
    }
    if(ArmConstants.kUseFeedForward){
      pivotPIDOutput += ArmConstants.kG*Math.cos(getCurrentPivotRotation(true)-(Math.PI/2));
    }

    _leftPivotController.set(pivotPIDOutput);
    _rightPivotController.set(pivotPIDOutput);
  }

  private void extensionPeriodic(){
    double extensionPIDOutput = _extensionPID.calculate(_currentExtensionDistance, _desiredExtensionDistance);
    SmartDashboard.putNumber("extensionPIDOutput", extensionPIDOutput);
    double difference = _desiredExtensionDistance - _currentExtensionDistance;

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


  public void setDesiredPivotRot(double _desiredRotation){
    _desiredPivotRotation = _desiredRotation;
  }


  //in inches
  public void setDesiredExtension(double _extension){
    _desiredExtensionDistance = _extension;
  }


  public boolean atAngleSetpoint(){
    return Math.abs(_desiredPivotRotation - _currentPivotRotation) < Units.degreesToRadians(4);
  }


  public boolean atTelescopeSetpoint(){
    return Math.abs(_desiredExtensionDistance - _currentExtensionDistance) < 0.5;
  }


  public Command waitUntilSpPivot(double sp){
    return new FunctionalCommand(()->setDesiredPivotRot(sp), ()->new PrintCommand("getName()"), (Boolean bool)->new PrintCommand("Finished Pivot"), this::atAngleSetpoint, this);
  }


  public Command waitUntilSpTelescope(double sp){
    return new FunctionalCommand(()->setDesiredExtension(sp), ()->new PrintCommand("finished"), (Boolean bool)-> new PrintCommand("getName()"), this::atTelescopeSetpoint, this);
  }
  

  public double getCurrentPivotRotation(boolean inRadians){
    double rotation;

    rotation = (_leftPivotController.getSelectedSensorPosition()) * ArmConstants.kEncoderResolution * ArmConstants.kFalconToFinalGear + this._pivotRelEncoderOffsetRot;
    
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
    this._desiredPivotRotation += i;
  }


  public void changeDesiredExtension(double i){
    this._desiredExtensionDistance += i;
  }


  private double getPivotAbsEncoderAngleRot(){
    return -_absPivEncoder.getAbsolutePosition() * ArmConstants.kPivotAbsEncToRotation + this._pivotRelEncoderOffsetRot;
  }

  private void updateModes(){
    SmartDashboard.updateValues();
    if(chooser.getSelected() == ArmMotorMode.BRAKE){
      _rightPivotController.setNeutralMode(NeutralMode.Brake);
      _leftPivotController.setNeutralMode(NeutralMode.Brake);
      extensionController.setIdleMode(IdleMode.kBrake);
    }
    else if(chooser.getSelected() == ArmMotorMode.COAST){
      _rightPivotController.setNeutralMode(NeutralMode.Coast);
      _leftPivotController.setNeutralMode(NeutralMode.Coast);
      extensionController.setIdleMode(IdleMode.kCoast);
    }
  } 

  private void resetEncoders(){
    _rightPivotController.setSelectedSensorPosition(0);
    _leftPivotController.setSelectedSensorPosition(0);
    extensionEncoder.setPosition(0);

    this._pivotRelEncoderOffsetRot = getPivotAbsEncoderAngleRot();
    this._currentPivotRotation = Units.rotationsToRadians(this._pivotRelEncoderOffsetRot);
    this._currentPivotRotation = this._desiredPivotRotation;
  }
}