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
  
  
  private final WPI_TalonFX mLeftPivotController = new WPI_TalonFX(ArmConstants.LEFT_PIV_MOTOR_ID);
  private final WPI_TalonFX mRightPivotController = new WPI_TalonFX(ArmConstants.RIGHT_PIV_MOTOR_ID);
  private final CANSparkMax mExtensionController = new CANSparkMax(ArmConstants.EXT_SPARK_ID, MotorType.kBrushless);

  private final DutyCycleEncoder mAbsPivEncoder = new DutyCycleEncoder(ArmConstants.DIOPORTPIV);
  private final RelativeEncoder mExtensionEncoder = mExtensionController.getEncoder();

  private final SlewRateLimiter mPivotRateLimiter;
  private final PIDController mExtensionPID;
  private final PIDController mPivotPID;

  // This is called Ultra Instinct because setting this boolean to true removes the clamps
  // so that if the belt skips, the operator can still run manually without the bad offsets
  private boolean mUltraInstinct = false;

  private double mPivotRelEncoderOffsetRot;
  private double mCurrentPivotRotation;
  private double mDesiredPivotRotation = ArmConstants.MIN_PIV_ANGLE_RAD;

  private double mCurrentExtensionDistance = ArmConstants.ZERO_EXTENSION_IN;
  private double mDesiredExtensionDistance = ArmConstants.MIN_EXT_LEN_IN;



  private boolean mIsInitialized = false;

  private final SendableChooser<ArmMotorMode> mChooser = new SendableChooser<>();


  public ArmControlSubsystem() {

    mPivotPID = new PIDController(1.4, 0, 0);
    mPivotPID.setTolerance(ArmConstants.RESTING_PIV_TOLERANCE_DEG);

    mExtensionPID = new PIDController(0.19, 0, 0);
    mExtensionPID.setTolerance(ArmConstants.RESTING_EXT_TOLERANCE_IN);

    mPivotRateLimiter = new SlewRateLimiter(ArmConstants.MAX_PIV_RATE_RAD_SEC);

    mAbsPivEncoder.setConnectedFrequencyThreshold(ArmConstants.CONNECTION_THRESH_HZ);
   
    this.mPivotRelEncoderOffsetRot = -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION + ArmConstants.PIV_INIT_OFFSET_ROT;
   
    setDefaultConfig();

    SmartDashboard.putNumber("kG", ArmConstants.KG);
    
    mChooser.addOption("Brake", ArmMotorMode.BRAKE);
    mChooser.addOption("Coast", ArmMotorMode.COAST); 
    mChooser.setDefaultOption("Brake", ArmMotorMode.BRAKE);

    SmartDashboard.putData("ArmMotorMode", mChooser);
    
  }

  public void setDefaultConfig(){
    mRightPivotController.configFactoryDefault();
    mLeftPivotController.configFactoryDefault();

    mRightPivotController.follow(mLeftPivotController);

    mRightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    mLeftPivotController.setInverted(ArmConstants.LEFT_PIV_MOTOR_INVERTED);

    this.updateModes();

    mRightPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mLeftPivotController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    mLeftPivotController.setSelectedSensorPosition(0);
    mRightPivotController.setSelectedSensorPosition(0);
    
    mExtensionController.setInverted(false);

    mExtensionEncoder.setPosition(0);

    mExtensionController.burnFlash();
  }

  @Override
  public void periodic() { 
    if(mChooser.getSelected() == ArmMotorMode.BRAKE && mIsInitialized){
      pivotPeriodic();
      extensionPeriodic();
    }else if(mChooser.getSelected() == ArmMotorMode.COAST){
      mDesiredPivotRotation = mCurrentPivotRotation;
      mDesiredExtensionDistance = mCurrentExtensionDistance;
    }

    if(!mIsInitialized && -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION != 0 && mAbsPivEncoder.isConnected()){
      resetEncoders();   
      mIsInitialized = true;
    }
    
    updateModes();
  

    // Getting Current And Desired Distances
    SmartDashboard.putNumber("CurrentExtension", this.getCurrentExtensionIn());
    SmartDashboard.putNumber("CurrentPivotDeg", Units.radiansToDegrees(mCurrentPivotRotation));

    if (CompConstants.DEBUG_MODE) {
      //Encoder Positions
      SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));
      SmartDashboard.putNumber("InitialAbsPivot", this.mPivotRelEncoderOffsetRot);

      SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
      SmartDashboard.putNumber("extensionEncoderPos", mExtensionEncoder.getPosition()*ArmConstants.EXTENSION_ROTATION_TO_INCHES);
    
      //Abs Encoder Debugging
      SmartDashboard.putNumber("pivotfreq", mAbsPivEncoder.getFrequency());
      SmartDashboard.putBoolean("IsInititilized", mIsInitialized);
      SmartDashboard.putBoolean("absreconnectioned", mAbsPivEncoder.isConnected());
      SmartDashboard.putNumber("AbsPivot", -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION);

      //Setpoint Debugging
      SmartDashboard.putBoolean("AtAnglePoint", this.atAngleSetpoint());
      SmartDashboard.putBoolean("AtExtensionPoint", this.atTelescopeSetpoint());
      SmartDashboard.putNumber("DesiredPivotDeg", Units.radiansToDegrees(mDesiredPivotRotation));
      SmartDashboard.putNumber("DesiredExtension", mDesiredExtensionDistance);


    }
    
    mDesiredPivotRotation = MathUtil.clamp(mDesiredPivotRotation, ArmConstants.MIN_PIV_ANGLE_RAD, ArmConstants.MAX_PIV_ANGLE_RAD);
    mCurrentPivotRotation = getCurrentPivotRotation(true);

    if (!mUltraInstinct) {
      mDesiredExtensionDistance = MathUtil.clamp(mDesiredExtensionDistance, ArmConstants.MIN_EXT_LEN_IN, ArmConstants.MAX_EXT_LEN_IN);
    }
    mCurrentExtensionDistance = getCurrentExtensionIn();
  }

  private void pivotPeriodic(){    
    double pivotPIDOutput = mPivotPID.calculate(mCurrentPivotRotation, mDesiredPivotRotation);

    //Desaturating PID output
    if(pivotPIDOutput > 0.55){
      pivotPIDOutput = 0.55;
    }else if(pivotPIDOutput < -0.55){
      pivotPIDOutput = -0.55;
    }

    if (CompConstants.DEBUG_MODE) {SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);}
    
    if (ArmConstants.RATE_LIMIT_ARM) {
      pivotPIDOutput = mPivotRateLimiter.calculate(pivotPIDOutput);
    }
    if(ArmConstants.ENABLE_FEEDFORWARD){
      pivotPIDOutput += ArmConstants.KG*Math.cos(getCurrentPivotRotation(true)-(Math.PI/2));
    }

    mLeftPivotController.set(pivotPIDOutput);
    mRightPivotController.set(pivotPIDOutput);
  }

  private void extensionPeriodic(){
    double extensionPIDOutput = mExtensionPID.calculate(mCurrentExtensionDistance, mDesiredExtensionDistance);
    SmartDashboard.putNumber("extensionPIDOutput", extensionPIDOutput);
    double difference = mDesiredExtensionDistance - mCurrentExtensionDistance;

    //Desaturating PID output
    if(extensionPIDOutput > .5){
      extensionPIDOutput = .5;
    }else if (extensionPIDOutput < -0.5){
      extensionPIDOutput = -.5;
    }

    //This is for handling the friction in the extension
    double offset = .005 * (difference > 0 ? 1 : -1);
    if (Math.abs(difference) > .14){
      mExtensionController.set(offset + extensionPIDOutput);
    }else{
      mExtensionController.set(0);
    }
  }


  public void setDesiredPivotRot(double desiredRotation){
    mDesiredPivotRotation = desiredRotation;
  }


  //in inches
  public void setDesiredExtension(double extension){
    mDesiredExtensionDistance = extension;
  }


  public boolean atAngleSetpoint(){
    return Math.abs(mDesiredPivotRotation - mCurrentPivotRotation) < Units.degreesToRadians(4);
  }


  public boolean atTelescopeSetpoint(){
    return Math.abs(mDesiredExtensionDistance - mCurrentExtensionDistance) < 0.5;
  }


  public Command waitUntilSpPivot(double sp){
    return new FunctionalCommand(()->setDesiredPivotRot(sp), ()->new PrintCommand("getName()"), (Boolean bool)->new PrintCommand("Finished Pivot"), this::atAngleSetpoint, this);
  }


  public Command waitUntilSpTelescope(double sp){
    return new FunctionalCommand(()->setDesiredExtension(sp), ()->new PrintCommand("finished"), (Boolean bool)-> new PrintCommand("getName()"), this::atTelescopeSetpoint, this);
  }
  

  public double getCurrentPivotRotation(boolean inRadians){
    double rotation;

    rotation = (mLeftPivotController.getSelectedSensorPosition()) * ArmConstants.ENCODER_RES * ArmConstants.PIV_MOTOR_TO_GEAR_ROT + this.mPivotRelEncoderOffsetRot;
    
    if(inRadians){
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }


  //convert encoder rotations to distance inches
  public double getCurrentExtensionIn(){
    return mExtensionEncoder.getPosition()*ArmConstants.GEAR_RATIO_EXTENSION_IN;
  }

  
  public void changeDesiredPivotRotation(double i){
    this.mDesiredPivotRotation += i;
  }


  public void changeDesiredExtension(double i){
    this.mDesiredExtensionDistance += i;
  }

  public boolean getmUltraInstinct(){
    return mUltraInstinct;
  }
  public void setmUltraInstinct(boolean toggle){
    mUltraInstinct = toggle;
  }


  private double getPivotAbsEncoderAngleRot(){
    return -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION + this.mPivotRelEncoderOffsetRot;
  }

  private void updateModes(){
    SmartDashboard.updateValues();
    if(mChooser.getSelected() == ArmMotorMode.BRAKE){
      mRightPivotController.setNeutralMode(NeutralMode.Brake);
      mLeftPivotController.setNeutralMode(NeutralMode.Brake);
      mExtensionController.setIdleMode(IdleMode.kBrake);
    }
    else if(mChooser.getSelected() == ArmMotorMode.COAST){
      mRightPivotController.setNeutralMode(NeutralMode.Coast);
      mLeftPivotController.setNeutralMode(NeutralMode.Coast);
      mExtensionController.setIdleMode(IdleMode.kCoast);
    }
  } 

  private void resetEncoders(){
    mRightPivotController.setSelectedSensorPosition(0);
    mLeftPivotController.setSelectedSensorPosition(0);
    mExtensionEncoder.setPosition(0);

    this.mPivotRelEncoderOffsetRot = getPivotAbsEncoderAngleRot();
    this.mCurrentPivotRotation = Units.rotationsToRadians(this.mPivotRelEncoderOffsetRot);
    this.mCurrentPivotRotation = this.mDesiredPivotRotation;
  }
}