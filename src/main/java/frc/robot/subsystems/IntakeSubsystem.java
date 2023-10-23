// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax _intakeMotor = new CANSparkMax(29,MotorType.kBrushless);

  public IntakeSubsystem() {
    _intakeMotor.setIdleMode(IdleMode.kBrake);
    _intakeMotor.burnFlash();
  
  }
  public void runIntake(double percentOutput){
    
   _intakeMotor.set(percentOutput);

  }


  @Override
  public void periodic() {
    // Intake Cone Outtake Cube
    if (Input.getRightTrigger() > IntakeConstants.kIntakeDeadZone) {runIntake(Input.getRightTrigger()/1.75);}
    
    // Intake Cube Outtake Cone
    if (Input.getLeftTrigger()>IntakeConstants.kIntakeDeadZone){runIntake(-Input.getLeftTrigger()/1.2);}

    //Set Intake to Stall Speed if Neither
    if (Input.getLeftTrigger()<IntakeConstants.kIntakeDeadZone && Input.getRightTrigger()<IntakeConstants.kIntakeDeadZone){runIntake(IntakeConstants.kIntakeStallSpeed);}
  }

}
