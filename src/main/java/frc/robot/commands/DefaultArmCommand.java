// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.Input;

public class DefaultArmCommand extends CommandBase {
  
  ArmControlSubsystem armControlSubsystem;

  public DefaultArmCommand(ArmControlSubsystem armControlSubsystem) {
    this.armControlSubsystem = armControlSubsystem;
    addRequirements(armControlSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    handleInput();
  }

  void handleInput(){

    // if(Input.getDPad() == Input.DPADUP){
    //   armControlSubsystem.changeDesiredPivotRotation(.025); //
    // }else if(Input.getDPad() == Input.DPADDOWN){
    //   armControlSubsystem.changeDesiredPivotRotation(-.025);

    // }

    if(Input.getLeftStickY() > 0.15){
      armControlSubsystem.changeDesiredPivotRotation(.03 * (Input.getLeftStickY()-0.15));
    }else if(Input.getLeftStickY() < -0.15){
      armControlSubsystem.changeDesiredPivotRotation(.03 * (Input.getLeftStickY()+0.15));
    }

    if(Input.getRightStickY() > 0.05){
      armControlSubsystem.changeDesiredExtension(.16 * (Input.getRightStickY() - 0.05));
    }else if(Input.getRightStickY() < -0.05){
      armControlSubsystem.changeDesiredExtension(.16 * (Input.getRightStickY() + 0.05));
    }
    
    // else if(Input.getDPad() == Input.DPADRIGHT){
    //   armControlSubsystem.changeDesiredExtension(.3);
    // }
    // else if(Input.getDPad() == Input.DPADLEFT){
    //   armControlSubsystem.changeDesiredExtension(-.3);
    // }
    
    else if(Input.getA()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.angleLevelsDeg[0]))),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.extensionLevelsIn[0]))

      );
      command.schedule();
    }
    else if(Input.getB()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.angleLevelsDeg[1]))),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.extensionLevelsIn[1]))

      );
      command.schedule();
    }
    else if(Input.getX()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.angleLevelsDeg[2]))),
        new WaitCommand(.5),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.extensionLevelsIn[2]))

      );
      command.schedule();
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}