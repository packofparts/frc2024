// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_TalonFX _intakeMotor = new WPI_TalonFX(ArmConstants.kIntakeID);

  public IntakeSubsystem() {
    _intakeMotor.setInverted(ArmConstants.kIntakeInverted);
  }
  public void runIntake(double percentOutput){
    _intakeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    if (Input.getRightTrigger()>ArmConstants.kIntakeDeadZone){runIntake(Input.getRightTrigger());}
    
    else if (Input.getLeftTrigger()>ArmConstants.kIntakeDeadZone){runIntake(-Input.getLeftTrigger());}

    else{runIntake(ArmConstants.kIntakeStallSpeed);}
  }
}
