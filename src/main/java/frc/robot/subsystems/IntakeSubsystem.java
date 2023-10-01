// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {
  private final WPI_TalonSRX _intakeMotor = new WPI_TalonSRX(ArmConstants.kIntakeID);

  public IntakeSubsystem() {
  
  }
  public void runIntake(double percentOutput){

    _intakeMotor.set(ControlMode.PercentOutput, percentOutput);

   // SmartDashboard.putData("Intake data", new Double(percentOutput));

  }

  @Override
  public void periodic() {
    // _intakeMotor.set(ControlMode.PercentOutput, -1);
    if (Input.getRightTrigger() > ArmConstants.kIntakeDeadZone) {runIntake(Input.getRightTrigger());}
    
    if (Input.getLeftTrigger()>ArmConstants.kIntakeDeadZone){runIntake(-Input.getLeftTrigger());}

    if (Input.getLeftTrigger()<ArmConstants.kIntakeDeadZone && Input.getRightTrigger()<ArmConstants.kIntakeDeadZone){runIntake(-ArmConstants.kIntakeStallSpeed);}
  }

}
