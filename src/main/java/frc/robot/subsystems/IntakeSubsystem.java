// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax mIntakeMotor =
      new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);

  public IntakeSubsystem() {
    mIntakeMotor.setIdleMode(IdleMode.kBrake);
    mIntakeMotor.burnFlash();

  }

  public void runIntake(double percentOutput) {
    mIntakeMotor.set(percentOutput);
  }

  public Command timedIntake(double percentOutput, double timeSec) {
    return (new SequentialCommandGroup(
        new RunCommand(() -> runIntake(percentOutput)).withTimeout(timeSec),
        new InstantCommand(() -> runIntake(0))));
  }

}
