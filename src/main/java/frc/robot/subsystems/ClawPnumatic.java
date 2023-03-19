// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants.MiscNonConstants;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

public class ClawPnumatic extends SubsystemBase {
  /** Creates a new ClawPnumatic. */
  //DoubleSolenoid intakeSolenoid1;
  Solenoid intakeSolenoid1;
  Compressor phCompressor;
  PneumaticHub hub;
  TalonSRX intakeMotor;
  AnalogPotentiometer pressureSensor;


  public ClawPnumatic() {
    
    hub = new PneumaticHub();
    
    /*
    intakeSolenoid1 = hub.makeDoubleSolenoid(0,1);
    pressureSensor = new AnalogPotentiometer(1);
    phCompressor = hub.makeCompressor();
    
    phCompressor.enableAnalog(20, 60);
    */
    intakeMotor = new TalonSRX(IntakeConstants.clawPort);
    intakeSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, 0); //extend
    phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriveConstants.debug) {
    SmartDashboard.putBoolean("ValueSwitch", phCompressor.getPressureSwitchValue());
    SmartDashboard.putBoolean("CompressorEnabled", phCompressor.isEnabled());
    }
    SmartDashboard.putNumber("pressure", phCompressor.getPressure());
    SmartDashboard.putBoolean("Claw Closed", intakeSolenoid1.get());
    phCompressor.enableDigital();
    if (phCompressor.getPressure() > 60) phCompressor.disable();
    else if (phCompressor.getPressure() < 60) phCompressor.enableDigital();


    //toggle intake solenoids
    // if (Input.getIntake()){spinIntake(1);}
    // else if(Input.getOuttake()){spinOuttake(1);}

    if (Input.getRightBumper()){
      togglePneumatics();

    }


    if(Input.getRightTrigger()!= 0){
      
      spinIntake(Input.getRightTrigger());

    }
    else if (Input.getLeftTrigger()!=0){
      spinOuttake(Input.getLeftTrigger());

    }else{
      changeIntake(0);
    }

    //spinIntake(1);
  }

  public void setConfig(){
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(IntakeConstants.motorInverted);
  }

  public void togglePneumatics(){
    if(intakeSolenoid1.get()){
      intakeSolenoid1.set(false);
    }
    else{intakeSolenoid1.set(true);}
  }

  public void openPneumatics(){
    intakeSolenoid1.set(false);
  }
  public void closePneumatics(){
    intakeSolenoid1.set(true);
  }

  public void compression(){
    if (phCompressor.getPressure() > 60) phCompressor.disable();
    else if (phCompressor.getPressure() < 60) phCompressor.enableDigital();
  }
  public void changePneumatics(boolean closed){
    intakeSolenoid1.set(closed);
  }

  public void changeIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void spinIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void spinOuttake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -speed);
  }
  
}
