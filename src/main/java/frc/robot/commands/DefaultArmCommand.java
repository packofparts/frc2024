// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    if(Input.getDPad() == Input.DPADUP){
      armControlSubsystem.changeDesiredPivotRotation(.5); //
    }else if(Input.getDPad() == Input.DPADDOWN){
      armControlSubsystem.changeDesiredPivotRotation(-.5);
    }else if(Input.getA()){
      armControlSubsystem.setDesiredExtension(Constants.extensionLevels[0]); // have to subtract the initial
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(Constants.angleLevels[0]));
    }else if(Input.getB()){
      armControlSubsystem.setDesiredExtension(Constants.extensionLevels[1]);
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(Constants.angleLevels[1]));
    }else if(Input.getX()){
      armControlSubsystem.setDesiredExtension(Constants.extensionLevels[2]);
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(Constants.angleLevels[2]));
    }else if(Input.getY()){
      armControlSubsystem.setDesiredExtension(Constants.minExtension);
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(Constants.minAngle));
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