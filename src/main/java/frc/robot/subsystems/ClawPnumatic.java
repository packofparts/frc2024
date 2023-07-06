// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CompConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.AutoMapConstants.GamePiece;

public class ClawPnumatic extends SubsystemBase {
  /** Creates a new ClawPnumatic. */
  //DoubleSolenoid intakeSolenoid1;
  Solenoid intakeSolenoid;
  Compressor phCompressor;
  PneumaticHub hub;
  TalonSRX intakeMotor;
  AnalogPotentiometer pressureSensor;


  double desiredSpeed = 0; 
  // use setIntake to instead set the desiredSpeed above
  // then constantly set the desiredSpeed to motors
  // then check if the encoder is velocity is above a certain velocity,
  // if it isn't lower the speed because that means a CUBE inside

  public ClawPnumatic() {
    
    hub = new PneumaticHub();
    
    intakeMotor = new TalonSRX(IntakeConstants.clawPort);
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0); //extend
    phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  


    this.closePneumatics();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    if (CompConstants.debug) {
      SmartDashboard.putBoolean("ValueSwitch", phCompressor.getPressureSwitchValue());
      SmartDashboard.putBoolean("CompressorEnabled", phCompressor.isEnabled());
    }

    SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
    SmartDashboard.putBoolean("Claw Closed", intakeSolenoid.get());


    
    //phCompressor.enableAnalog(100, 105); // TODO: Move to initialize method



    // if (Input.getRightBumper()){
    //   this.togglePneumatics();
    // }

    // if(Input.getRightTrigger() != 0){
    //   setIntake(Input.getRightTrigger());
    // }
    // else if (Input.getLeftTrigger() != 0){
    //   setIntake(-Input.getLeftTrigger());
    // } 
    // else {
    //   setIntake(0.2);
    // }
  }

  public void setConfig(){
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(IntakeConstants.motorInverted);
  }

  public void togglePneumatics(){
    if(intakeSolenoid.get()){
      intakeSolenoid.set(false);
    }
    else{intakeSolenoid.set(true);}
  }

  public void openPneumatics(){
    intakeSolenoid.set(true); //these were previously switched around
  }
  public void closePneumatics(){
    intakeSolenoid.set(false);
  }


  public void changePneumatics(boolean closed){
    intakeSolenoid.set(closed);
  }

  public void setIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void spinIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void spinOuttake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -speed);
  }
  public Command dropPiece(GamePiece piece){
    switch (piece){
      case CUBE:
        return new SequentialCommandGroup(new InstantCommand(() -> this.openPneumatics(), this),new InstantCommand(()->spinIntake(-0.3),this),new WaitCommand(0.4));
      case CONE:
        return new SequentialCommandGroup(new InstantCommand(() -> this.openPneumatics(), this),new WaitCommand(0.7), new InstantCommand(() -> this.closePneumatics(), this));
      default:
        return null;
    }
  }
  
}
