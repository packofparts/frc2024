// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClawMotor extends SubsystemBase {
  /** Creates a new ClawMotor. */
  CANSparkMax main;
  public ClawMotor() {
    main = new CANSparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Input.getIntake()){
      intake();
    }
    if(Input.getOuttake()){
      outtake();
    }
    else{
      brake();
    }

  }
  public void intake(){
    main.set(1);
  }
  public void outtake(){
    main.set(-1);
  }
  public void brake(){
    main.set(0);
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
