// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armcontrolcmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;
import frc.robot.subsystems.ArmControlSubsystem.MoveArmConfig;

public class FullArmContolCmd extends CommandBase {
  
  ArmControlSubsystem arm;
  ClawPnumatic claw;
  
  double desiredPivot, desiredExtension, intakeSpeed;
  boolean pneumatics;

  MoveArmConfig config;


  Command command;

  public FullArmContolCmd(ArmControlSubsystem arm, ClawPnumatic claw, double desiredPivot, double desiredExtension, double intakeSpeed, boolean pneumatics, MoveArmConfig moveArmConfig) {
    
    this.arm = arm;
    this.claw = claw;
    this.desiredPivot = desiredPivot;
    this.desiredExtension = desiredExtension;
    this.intakeSpeed = intakeSpeed;
    this.pneumatics = pneumatics;
    this.config = moveArmConfig;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    if(this.config == MoveArmConfig.SIMULTANEOUS){
      command = new ParallelCommandGroup(new PivotCmd(arm, desiredPivot), new ExtensionCmd(arm, desiredExtension), new ClawCmd(claw, this.pneumatics, intakeSpeed));
    }



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
