// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants.MiscNonConstants;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PnumaticConstants;

public class ClawPnumatic extends SubsystemBase {
  /** Creates a new ClawPnumatic. */
  Solenoid intakeSolenoid1;
  Compressor phCompressor;
  PneumaticHub hub;
  public ClawPnumatic() {
    intakeSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, PnumaticConstants.intakeSolenoid1ID);
    phCompressor = new Compressor(PnumaticConstants.compressorID, PneumaticsModuleType.REVPH);
    hub = new PneumaticHub();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (phCompressor.getPressure() > 119) phCompressor.disable();
    else if (phCompressor.getPressure() < 118) phCompressor.enableDigital();


    //toggle intake solenoids
    if (Input.getIntake()){togglePneumatics();}
  }

  public void togglePneumatics(){
    if(intakeSolenoid1.get()){intakeSolenoid1.set(false);}
    else{intakeSolenoid1.set(true);}
  }
}
