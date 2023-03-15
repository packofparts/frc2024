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
import frc.robot.Constants.MiscNonConstants;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ClawPnumatic extends SubsystemBase {
  /** Creates a new ClawPnumatic. */
  Solenoid intakeSolenoid1;
  Compressor phCompressor;
  TalonSRX intakeMotor;

  public ClawPnumatic() {
    intakeSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, IntakeConstants.intakeSolenoid1ID);
    phCompressor = new Compressor(IntakeConstants.compressorID, PneumaticsModuleType.REVPH);
    intakeMotor = new TalonSRX(IntakeConstants.intakePort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (phCompressor.getPressure() > 60) phCompressor.disable();
    else if (phCompressor.getPressure() <= 60) phCompressor.enableDigital();


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

    }
  }

  public void setConfig(){
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(IntakeConstants.motorInverted);
  }

  public void togglePneumatics(){
    if(intakeSolenoid1.get()){intakeSolenoid1.set(false);}
    else{intakeSolenoid1.set(true);}
  }

  public void openPneumatics(){
    intakeSolenoid1.set(true);
  }
  public void closePneumatics(){
    intakeSolenoid1.set(false);
  }

  public void spinIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void spinOuttake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -speed);
  }
  
}
