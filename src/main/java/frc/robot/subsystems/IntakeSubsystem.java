// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax _intakeMotor = new CANSparkMax(29,MotorType.kBrushless);

  public IntakeSubsystem() {
    _intakeMotor.setIdleMode(IdleMode.kBrake);
    _intakeMotor.burnFlash();
    //_intakeMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  
  }
  public void runIntake(double percentOutput){
    
   _intakeMotor.set(percentOutput);

   // SmartDashboard.putData("Intake data", new Double(percentOutput));

  }


  @Override
  public void periodic() {
    // Intake Cone Outtake Cube
    if (Input.getRightTrigger() > ArmConstants.kIntakeDeadZone) {runIntake(Input.getRightTrigger()/1.75);}
    
    // Intake Cube Outtake Cone
    if (Input.getLeftTrigger()>ArmConstants.kIntakeDeadZone){runIntake(-Input.getLeftTrigger()/1.2);}

    if (Input.getLeftTrigger()<ArmConstants.kIntakeDeadZone && Input.getRightTrigger()<ArmConstants.kIntakeDeadZone){runIntake(ArmConstants.kIntakeStallSpeed);}
  }

}