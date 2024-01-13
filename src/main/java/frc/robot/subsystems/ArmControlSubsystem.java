package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private final TalonFX mLeftPivotController = new TalonFX(ArmConstants.LEFT_PIV_MOTOR_ID);
  private final TalonFX mRightPivotController = new TalonFX(ArmConstants.RIGHT_PIV_MOTOR_ID);
  private final DutyCycleEncoder mAbsPivEncoder = new DutyCycleEncoder(ArmConstants.DIOPORTPIV);
  private final TalonFXConfiguration mLeftTalonFXConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration mRighTalonFXConfiguration = new TalonFXConfiguration();
  private final SlewRateLimiter mPivotRateLimiter;
  private final PIDController mPivotPID;

  // This is called Ultra Instinct because setting this boolean to true removes the clamps
  // so that if the belt skips, the operator can still run manually without the bad offsets
  private boolean mUltraInstinct = false;

  private double mPivotRelEncoderOffsetRot;
  private double mCurrentPivotRotation;
  private double mDesiredPivotRotation = ArmConstants.MIN_PIV_ANGLE_RAD;

  private boolean mIsInitialized = false;
  private final DutyCycleOut mDutyCycleCommand = new DutyCycleOut(0).withUpdateFreqHz(50);

  private final SendableChooser<ArmMotorMode> mChooser = new SendableChooser<>();


  public ArmControlSubsystem() {

    mChooser.addOption("Brake", ArmMotorMode.BRAKE);
    mChooser.addOption("Coast", ArmMotorMode.COAST);
    mChooser.setDefaultOption("Brake", ArmConstants.INITIAL_ARM_MOTOR_MODE);

    mPivotPID = new PIDController(0.8, 0, 0);
    mPivotPID.setTolerance(ArmConstants.RESTING_PIV_TOLERANCE_RAD);

    mPivotRateLimiter = new SlewRateLimiter(ArmConstants.MAX_PIV_RATE_RAD_SEC);

    mAbsPivEncoder.setConnectedFrequencyThreshold(ArmConstants.CONNECTION_THRESH_HZ);

    mPivotRelEncoderOffsetRot =
        -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.ABS_ENC_TO_FINAL_SPROCKET
            + ArmConstants.PIV_INIT_OFFSET_ROT;

    setDefaultMotorConfig();
  }

  public void setDefaultMotorConfig() {

    mRightPivotController.setControl(new Follower(mLeftPivotController.getDeviceID(), true));
    mLeftPivotController.setInverted(ArmConstants.LEFT_PIV_MOTOR_INVERTED);

    updateModes();
  }

  @Override
  public void periodic() {
    if (mChooser.getSelected() == ArmMotorMode.BRAKE && mIsInitialized) {
      pivotPeriodic();
    } else if (mChooser.getSelected() == ArmMotorMode.COAST) {
      mDesiredPivotRotation = mCurrentPivotRotation;
    }

    if (!mIsInitialized && mAbsPivEncoder.isConnected()) {
      resetEncoders();
      mIsInitialized = true;
    }

    updateModes();


    // Getting Current And Desired Distances
    SmartDashboard.putNumber("CurrentPivotDeg", Units.radiansToDegrees(mCurrentPivotRotation));

    if (CompConstants.DEBUG_MODE) {

      SmartDashboard.putData("ArmMotorMode", mChooser);
      // Encoder Positions
      SmartDashboard.putNumber("PivotPos", getCurrentPivotRotation(false));
      SmartDashboard.putNumber("InitialAbsPivot", mPivotRelEncoderOffsetRot);

      // Abs Encoder Debugging
      SmartDashboard.putNumber("pivotfreq", mAbsPivEncoder.getFrequency());
      SmartDashboard.putBoolean("IsInititilized", mIsInitialized);
      SmartDashboard.putBoolean("absreconnectioned", mAbsPivEncoder.isConnected());
      SmartDashboard.putNumber("AbsPivot",
          -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.ABS_ENC_TO_FINAL_SPROCKET);

      // Setpoint Debugging
      SmartDashboard.putBoolean("AtAnglePoint", atAngleSetpoint());
      SmartDashboard.putNumber("DesiredPivotDeg", Units.radiansToDegrees(mDesiredPivotRotation));

      SmartDashboard.putNumber("pivot error", mDesiredPivotRotation - mCurrentPivotRotation);


    }

    mDesiredPivotRotation = MathUtil.clamp(mDesiredPivotRotation, ArmConstants.MIN_PIV_ANGLE_RAD,
        ArmConstants.MAX_PIV_ANGLE_RAD);
    mCurrentPivotRotation = getCurrentPivotRotation(true);
  }

  private void pivotPeriodic() {
    double pivotPIDOutput = mPivotPID.calculate(mCurrentPivotRotation, mDesiredPivotRotation);

    // Desaturating PID output
    pivotPIDOutput = MathUtil.clamp(pivotPIDOutput, -ArmConstants.PIV_MAX_PID_CONTRIBUTION_PERCENT,
        ArmConstants.PIV_MAX_PID_CONTRIBUTION_PERCENT);



    if (ArmConstants.RATE_LIMIT_ARM) {
      pivotPIDOutput = mPivotRateLimiter.calculate(pivotPIDOutput);
    }
    if (ArmConstants.ENABLE_FEEDFORWARD) {
      double kgOutPut = ArmConstants.KG * Math.cos(mCurrentPivotRotation - (Math.PI / 2));
      // Desaturating kG output
      pivotPIDOutput += MathUtil.clamp(kgOutPut, -ArmConstants.PIV_MAX_KG_CONTRIBUTION_PERCENT,
          ArmConstants.PIV_MAX_KG_CONTRIBUTION_PERCENT);
    }

    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putNumber("pivotPIDOutput", pivotPIDOutput);
    }

    mLeftPivotController.setControl(mDutyCycleCommand.withOutput(MathUtil.clamp(pivotPIDOutput,
        -ArmConstants.PIV_MAX_SPEED_CLAMP_PERCENT, ArmConstants.PIV_MAX_SPEED_CLAMP_PERCENT)));

    mRightPivotController.setControl(mDutyCycleCommand.withOutput(MathUtil.clamp(pivotPIDOutput,
        -ArmConstants.PIV_MAX_SPEED_CLAMP_PERCENT, ArmConstants.PIV_MAX_SPEED_CLAMP_PERCENT)));
  }


  public void setDesiredPivotRot(double desiredRotation) {
    mDesiredPivotRotation = desiredRotation;
  }


  public boolean atAngleSetpoint() {
    return Math
        .abs(mDesiredPivotRotation - mCurrentPivotRotation) < ArmConstants.ACTIVE_PIV_TOLERANCE_RAD;
  }



  public Command waitUntilSpPivot(double sp) {
    return new FunctionalCommand(() -> setDesiredPivotRot(sp), () -> new PrintCommand("getName()"),
        (Boolean bool) -> new PrintCommand("Finished Pivot"), this::atAngleSetpoint, this);
  }


  public double getCurrentPivotRotation(boolean inRadians) {
    double rotation;
    rotation = mLeftPivotController.getRotorPosition().getValue() * ArmConstants.FALCON_TO_ABS_ENC
        * ArmConstants.ABS_ENC_TO_FINAL_SPROCKET;

    if (inRadians) {
      return rotation * 2 * Math.PI;
    }
    return rotation;
  }


  public void changeDesiredPivotRotation(double i) {
    mDesiredPivotRotation += i;
  }


  public boolean getmUltraInstinct() {
    return mUltraInstinct;
  }

  public void setmUltraInstinct(boolean toggle) {
    mUltraInstinct = toggle;
  }


  private double getPivotAbsEncoderAngleRot() {
    return -mAbsPivEncoder.getAbsolutePosition() * ArmConstants.ABS_ENC_TO_FINAL_SPROCKET
        + mPivotRelEncoderOffsetRot;
  }

  private void updateModes() {

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    if (mChooser.getSelected() == ArmMotorMode.BRAKE) {

      motorConfigs.NeutralMode = NeutralModeValue.Brake;

    } else if (mChooser.getSelected() == ArmMotorMode.COAST) {

      motorConfigs.NeutralMode = NeutralModeValue.Coast;
    }
    mLeftPivotController.getConfigurator().refresh(motorConfigs);

  }

  private void resetEncoders() {
    mPivotRelEncoderOffsetRot = getPivotAbsEncoderAngleRot();
    mCurrentPivotRotation = Units.rotationsToRadians(mPivotRelEncoderOffsetRot);
    mCurrentPivotRotation = mDesiredPivotRotation;
  }
}
