// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(29,MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.burnFlash();
  
  }
  public void runIntake(double percentOutput){
    
   intakeMotor.set(percentOutput);

  }

}
