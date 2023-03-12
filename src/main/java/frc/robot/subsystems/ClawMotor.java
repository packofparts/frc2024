// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;

public class ClawMotor extends SubsystemBase {
  /** Creates a new ClawMotor. */
  TalonSRX main;
  public ClawMotor() {
    main = new TalonSRX(ArmConstants.clawPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Input.getIntake()){
      intake();
    }
    else if(Input.getOuttake()){
      outtake();
    }
    else{
      brake();
    }

  }
  public void intake(){
    main.set(TalonSRXControlMode.PercentOutput, 1);;
  }
  public void outtake(){
    main.set(TalonSRXControlMode.PercentOutput,-1);
  }
  public void brake(){
    main.set(TalonSRXControlMode.PercentOutput,0.1);
  }


  public void setSpeed(double speed){
    main.set(TalonSRXControlMode.PercentOutput,speed);
  }

  /**
   * Returns a command for intaking a gamepiece
   * @param runtime total time in seconds for the intake to inttake on call of command before ending
   * @return
   */
  public Command getIntakeCmd(double runtime){
    return new InstantCommand(this::intake,this).andThen(new WaitCommand(runtime)).andThen(this::brake);
  }
  /**
   * Returns a command for outtaking a gamepiece
   * @param runtime total time in seconds for the intake to outtake on call of command before ending
   * @return
   */
  public Command getOuttakeCmd(double runtime){
    return new InstantCommand(this::intake,this).andThen(new WaitCommand(runtime)).andThen(this::brake);
  }
}
