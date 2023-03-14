// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    if(Input.getDPad() == Input.DPADUP){
      armControlSubsystem.changeDesiredPivotRotation(.025); //
    }else if(Input.getDPad() == Input.DPADDOWN){
      armControlSubsystem.changeDesiredPivotRotation(-.025);

    }
    
    else if(Input.getDPad() == Input.DPADRIGHT){
      armControlSubsystem.changeDesiredExtension(.12);
    }
    else if(Input.getDPad() == Input.DPADLEFT){
      armControlSubsystem.changeDesiredExtension(-.12);
    }
    
    else if(Input.getA()){
      armControlSubsystem.setDesiredExtension(ArmConstants.extensionLevelsIn[0]); // have to subtract the initial
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.angleLevelsDeg[0]));
    }else if(Input.getB()){
      armControlSubsystem.setDesiredExtension(ArmConstants.extensionLevelsIn[1]);
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.angleLevelsDeg[1]));
    }else if(Input.getX()){
      armControlSubsystem.setDesiredExtension(ArmConstants.extensionLevelsIn[2]);
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.angleLevelsDeg[2]));
    }else if(Input.getY()){
      armControlSubsystem.setDesiredExtension(ArmConstants.minExtensionIn);
      armControlSubsystem.setDesiredPivotRotation(Units.degreesToRadians(ArmConstants.minAngleRad));
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