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
    COAST, BRAKE, OFF
  }

  private final WPI_TalonFX mLeftPivotController = new WPI_TalonFX(ArmConstants.LEFT_PIV_MOTOR_ID);
  private final WPI_TalonFX mRightPivotController =
      new WPI_TalonFX(ArmConstants.RIGHT_PIV_MOTOR_ID);
  private final CANSparkMax mExtensionController =
      new CANSparkMax(ArmConstants.EXT_SPARK_ID, MotorType.kBrushless);

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

  private double mCurrentPivPID = 0.0;
  private double mPreviousPivotAngleRad = 0.0;

  private double mPreviousExtIn = 0.0;

  // if true, prevent movement in future periodic calls
  private static boolean mPivCoastOverride = false;
  private static boolean mExtCoastOverride = false;


  public ArmControlSubsystem() {

    mChooser.addOption("Brake", ArmMotorMode.BRAKE);
    mChooser.addOption("Coast", ArmMotorMode.COAST);
    mChooser.setDefaultOption("Brake", ArmConstants.INITIAL_ARM_MOTOR_MODE);

    mPivotPID = new PIDController(1.4, 0, 0);
    mPivotPID.setTolerance(ArmConstants.RESTING_PIV_TOLERANCE_RAD);

    mExtensionPID = new PIDController(0.19, 0, 0);
    mExtensionPID.setTolerance(ArmConstants.RESTING_EXT_TOLERANCE_IN);

    mPivotRateLimiter = new SlewRateLimiter(ArmConstants.MAX_PIV_RATE_RAD_SEC);

    mAbsPivEncoder.setConnectedFrequencyThreshold(ArmConstants.CONNECTION_THRESH_HZ);

    mPivotRelEncoderOffsetRot =
        -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION
            + ArmConstants.PIV_INIT_OFFSET_ROT;

    setDefaultMotorConfig();
  }

  public void setDefaultMotorConfig() {
    mRightPivotController.configFactoryDefault();
    mLeftPivotController.configFactoryDefault();

    mRightPivotController.follow(mLeftPivotController);

    mRightPivotController.setInverted(TalonFXInvertType.OpposeMaster);
    mLeftPivotController.setInverted(ArmConstants.LEFT_PIV_MOTOR_INVERTED);

    updateModes();

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
    if (mChooser.getSelected() == ArmMotorMode.BRAKE && mIsInitialized) {
      if (!mPivCoastOverride) {
        pivotPeriodic();
      }
      if (!mExtCoastOverride) {
        extensionPeriodic();
      }
    } else if (mChooser.getSelected() == ArmMotorMode.COAST) {
      mDesiredPivotRotation = mCurrentPivotRotation;
      mDesiredExtensionDistance = mCurrentExtensionDistance;
    }

    if (!mIsInitialized && Double.compare(mCurrentPivotRotation, mCurrentExtensionDistance) == 0
        && mAbsPivEncoder.isConnected()) {
      resetEncoders();
      mIsInitialized = true;
    }

    updateModes();


    // Getting Current And Desired Distances
    SmartDashboard.putNumber("CurrentExtension", getCurrentExtensionIn());
    SmartDashboard.putNumber("CurrentPivotDeg", Units.radiansToDegrees(mCurrentPivotRotation));

    if (CompConstants.DEBUG_MODE) {

      SmartDashboard.putData("ArmMotorMode", mChooser);
      // Encoder Positions
      SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));
      SmartDashboard.putNumber("InitialAbsPivot", mPivotRelEncoderOffsetRot);

      SmartDashboard.putNumber("extensionSensorOutput", getCurrentExtensionIn());
      SmartDashboard.putNumber("extensionEncoderPos",
          mExtensionEncoder.getPosition() * ArmConstants.EXTENSION_ROTATION_TO_INCHES);

      // Abs Encoder Debugging
      SmartDashboard.putNumber("pivotfreq", mAbsPivEncoder.getFrequency());
      SmartDashboard.putBoolean("IsInititilized", mIsInitialized);
      SmartDashboard.putBoolean("absreconnectioned", mAbsPivEncoder.isConnected());
      SmartDashboard.putNumber("AbsPivot",
          -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION);

      // Setpoint Debugging
      SmartDashboard.putBoolean("AtAnglePoint", atAngleSetpoint());
      SmartDashboard.putBoolean("AtExtensionPoint", atTelescopeSetpoint());
      SmartDashboard.putNumber("DesiredPivotDeg", Units.radiansToDegrees(mDesiredPivotRotation));
      SmartDashboard.putNumber("DesiredExtension", mDesiredExtensionDistance);


    }

    mDesiredPivotRotation = MathUtil.clamp(mDesiredPivotRotation, ArmConstants.MIN_PIV_ANGLE_RAD,
        ArmConstants.MAX_PIV_ANGLE_RAD);
    mCurrentPivotRotation = getCurrentPivotRotation(true);

    if (!mUltraInstinct) {
      mDesiredExtensionDistance = MathUtil.clamp(mDesiredExtensionDistance,
          ArmConstants.MIN_EXT_LEN_IN, ArmConstants.MAX_EXT_LEN_IN);
      if (mDesiredPivotRotation < ArmConstants.MAX_EXT_TOUCHES_GROUND_ANGLE_RAD) {
        mDesiredExtensionDistance =
            MathUtil.clamp(mDesiredExtensionDistance, ArmConstants.MIN_EXT_LEN_IN,
                ArmConstants.PIV_POS_Y_IN / Math.cos(mCurrentPivotRotation));
      } else {
        mDesiredExtensionDistance = MathUtil.clamp(mDesiredExtensionDistance,
            ArmConstants.MIN_EXT_LEN_IN, ArmConstants.MAX_EXT_LEN_IN);
      }
    }
    mCurrentExtensionDistance = getCurrentExtensionIn();
  }

  private void pivotPeriodic() {
    double pivotPIDOutput = mPivotPID.calculate(mCurrentPivotRotation, mDesiredPivotRotation);

    // Desaturating PID output
    pivotPIDOutput = MathUtil.clamp(pivotPIDOutput, -ArmConstants.PIV_MAX_PID_CONTRIBUTION_PERCENT,
        ArmConstants.PIV_MAX_PID_CONTRIBUTION_PERCENT);

    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);
    }

    if (ArmConstants.RATE_LIMIT_ARM) {
      pivotPIDOutput = mPivotRateLimiter.calculate(pivotPIDOutput);
    }
    if (ArmConstants.ENABLE_FEEDFORWARD) {
      double kgOutPut = ArmConstants.KG * Math.cos(getCurrentPivotRotation(true) - (Math.PI / 2));
      // Desaturating kG output
      pivotPIDOutput += MathUtil.clamp(kgOutPut, -ArmConstants.PIV_MAX_KG_CONTRIBUTION_PERCENT,
          ArmConstants.PIV_MAX_KG_CONTRIBUTION_PERCENT);
    }
    double expectedDistance = ArmConstants.MAX_PIV_RATE_RAD_SEC * mCurrentPivPID * 0.02; // 0.02 =
    // 20 ms

    // comparing the difference (between predicted distance traveled and the actual distance
    // traveled) with the tolerance
    if ((Math.abs(mCurrentPivotRotation - mPreviousPivotAngleRad)
        - expectedDistance) > ArmConstants.ACTIVE_PIV_TOLERANCE_RAD) { // outside tolerance value
      // turn off motors
      mLeftPivotController.setNeutralMode(NeutralMode.Coast);
      mRightPivotController.setNeutralMode(NeutralMode.Coast); // set motors to coast
      setExtOverride(true); // cancelling ability to move unless override cancel button is pressed
    } else { // inside tolerance value
      mPreviousPivotAngleRad = mCurrentPivotRotation;
      mCurrentPivPID = MathUtil.clamp(pivotPIDOutput, -ArmConstants.PIV_MAX_SPEED_CLAMP_PERCENT,
          ArmConstants.PIV_MAX_SPEED_CLAMP_PERCENT);
      mLeftPivotController.set(mCurrentPivPID);
      mRightPivotController.set(mCurrentPivPID);
    }
  }

  private void extensionPeriodic() {
    double extensionPIDOutput =
        mExtensionPID.calculate(mCurrentExtensionDistance, mDesiredExtensionDistance);
    double difference = mDesiredExtensionDistance - mCurrentExtensionDistance;

    // Desaturating PID output
    extensionPIDOutput =
        MathUtil.clamp(extensionPIDOutput, -ArmConstants.EXT_MAX_PID_CONTRIBUTION_PERCENT,
            ArmConstants.EXT_MAX_PID_CONTRIBUTION_PERCENT);
    SmartDashboard.putNumber("extensionPIDOutput", extensionPIDOutput);

    // This is for handling the friction in the extension
    double offset = ArmConstants.EXT_FRICTION_COEFF * (difference > 0 ? 1 : -1);
    SmartDashboard.putNumber("difference", offset);

    double expectedDistance = mExtensionController.getEncoder().getVelocity() * 0.02; // 20 ms

    // comparing the difference (between predicted distance traveled and the actual distance
    // traveled) with the tolerance
    if (Math.abs((mCurrentExtensionDistance - mPreviousExtIn) // outside tolerance value
        - expectedDistance) > ArmConstants.ACTIVE_EXT_TOLERANCE_IN) {
      mExtensionController.setIdleMode(IdleMode.kCoast); // set motors to coast
      setExtOverride(true); // cancelling ability to move unless override cancel button is pressed
    } else { // inside tolerance value
      if (Math.abs(difference) > ArmConstants.EXT_FRICTION_ACTIVATION_THRESH) {
        mExtensionController.set(MathUtil.clamp(offset + extensionPIDOutput,
            -ArmConstants.EXT_MAX_SPEED_CLAMP_PERCENT, ArmConstants.EXT_MAX_SPEED_CLAMP_PERCENT));
      } else {
        mExtensionController.set(0);
      }
    }
  }


  public void setDesiredPivotRot(double desiredRotation) {
    mDesiredPivotRotation = desiredRotation;
  }


  // in inches
  public void setDesiredExtension(double extension) {
    mDesiredExtensionDistance = extension;
  }


  public boolean atAngleSetpoint() {
    return Math
        .abs(mDesiredPivotRotation - mCurrentPivotRotation) < ArmConstants.ACTIVE_PIV_TOLERANCE_RAD;
  }


  public boolean atTelescopeSetpoint() {
    return Math.abs(mDesiredExtensionDistance
        - mCurrentExtensionDistance) < ArmConstants.ACTIVE_EXT_TOLERANCE_IN;
  }


  public Command waitUntilSpPivot(double sp) {
    return new FunctionalCommand(() -> setDesiredPivotRot(sp), () -> new PrintCommand("getName()"),
        (Boolean bool) -> new PrintCommand("Finished Pivot"), this::atAngleSetpoint, this);
  }


  public Command waitUntilSpTelescope(double sp) {
    return new FunctionalCommand(() -> setDesiredExtension(sp), () -> new PrintCommand("finished"),
        (Boolean bool) -> new PrintCommand("getName()"), this::atTelescopeSetpoint, this);
  }


  public double getCurrentPivotRotation(boolean inRadians) {
    double rotation;

    rotation = (mLeftPivotController.getSelectedSensorPosition()) * ArmConstants.PIVOT_ENCODER_RES
        * ArmConstants.PIV_MOTOR_TO_GEAR_ROT + mPivotRelEncoderOffsetRot;

    if (inRadians) {
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }


  // convert encoder rotations to distance inches
  public double getCurrentExtensionIn() {
    return mExtensionEncoder.getPosition() * ArmConstants.EXT_MOTOR_TO_BELT_IN;
  }


  public void changeDesiredPivotRotation(double i) {
    mDesiredPivotRotation += i;
  }


  public void changeDesiredExtension(double i) {
    mDesiredExtensionDistance += i;
  }

  public boolean getmUltraInstinct() {
    return mUltraInstinct;
  }

  public void setmUltraInstinct(boolean toggle) {
    mUltraInstinct = toggle;
  }

  // pivot is set to coast when true
  public static void setPivOverride(boolean override) {
    mPivCoastOverride = override;
  }

  // extension is set to coast when true
  public static void setExtOverride(boolean override) {
    mExtCoastOverride = override;
  }


  private double getPivotAbsEncoderAngleRot() {
    return -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.PIVOT_ABS_ENC_TO_ROTATION
        + mPivotRelEncoderOffsetRot;
  }

  private void updateModes() {
    SmartDashboard.updateValues();

    if (mChooser.getSelected() == ArmMotorMode.BRAKE) {
      mRightPivotController.setNeutralMode(NeutralMode.Brake);
      mLeftPivotController.setNeutralMode(NeutralMode.Brake);
      mExtensionController.setIdleMode(IdleMode.kBrake);
    } else if (mChooser.getSelected() == ArmMotorMode.COAST) {
      mRightPivotController.setNeutralMode(NeutralMode.Coast);
      mLeftPivotController.setNeutralMode(NeutralMode.Coast);
      mExtensionController.setIdleMode(IdleMode.kCoast);
    }
  }

  private void resetEncoders() {
    mRightPivotController.setSelectedSensorPosition(0);
    mLeftPivotController.setSelectedSensorPosition(0);
    mExtensionEncoder.setPosition(0);

    mPivotRelEncoderOffsetRot = getPivotAbsEncoderAngleRot();
    mCurrentPivotRotation = Units.rotationsToRadians(mPivotRelEncoderOffsetRot);
    mCurrentPivotRotation = mDesiredPivotRotation;
  }
}
