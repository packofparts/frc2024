// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.Input;

public class DefaultArmCommand extends CommandBase {
  
  private ArmControlSubsystem armControlSubsystem;

  public DefaultArmCommand(ArmControlSubsystem armControlSubsystem) {
    this.armControlSubsystem = armControlSubsystem;
    addRequirements(armControlSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    handleInput();
  }

  void handleInput(){

    if(Input.getLeftStickY() > 0.15){
      armControlSubsystem.changeDesiredPivotRotation(.05 * (Input.getLeftStickY()-0.15));
    }else if(Input.getLeftStickY() < -0.15){
      armControlSubsystem.changeDesiredPivotRotation(.05 * (Input.getLeftStickY()+0.15));
    }

    if(Input.getRightStickY() > 0.05){
      armControlSubsystem.changeDesiredExtension(.17 * (Input.getRightStickY() - 0.05));
    }else if(Input.getRightStickY() < -0.05){
      armControlSubsystem.changeDesiredExtension(.17 * (Input.getRightStickY() + 0.05));
    }
    
    
    else if(Input.getA()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRot(ArmConstants.ArmState.STOW.pivotAngleRad),armControlSubsystem),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.ArmState.STOW.extentionDistIn),armControlSubsystem)

      );
      command.schedule();
    }
    else if(Input.getB()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRot(ArmConstants.ArmState.MID_NODE_CONE.pivotAngleRad),armControlSubsystem),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.ArmState.MID_NODE_CONE.extentionDistIn),armControlSubsystem)

      );
      command.schedule();
    }
    else if(Input.getX()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        armControlSubsystem.waitUntilSpPivot(ArmConstants.ArmState.UPPER_NODE_CONE.pivotAngleRad),
        armControlSubsystem.waitUntilSpTelescope(ArmConstants.ArmState.UPPER_NODE_CONE.extentionDistIn)
      );
      command.schedule();
    }
    else if(Input.getY()){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRot(ArmConstants.ArmState.SUBSTATION_CONE.pivotAngleRad),armControlSubsystem),
        new WaitCommand(1),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.ArmState.SUBSTATION_CONE.extentionDistIn),armControlSubsystem)

      );
      command.schedule();
    }
    else if(Input.getDPad() == Input.DPADUP){
      SequentialCommandGroup command = new SequentialCommandGroup(
        new InstantCommand(()->armControlSubsystem.setDesiredPivotRot(ArmConstants.ArmState.GROUND_PICKUP_CONE.pivotAngleRad),armControlSubsystem),
        new WaitCommand(0.5),
        new InstantCommand(()->armControlSubsystem.setDesiredExtension(ArmConstants.ArmState.GROUND_PICKUP_CONE.extentionDistIn),armControlSubsystem)

      );
      command.schedule();
    }

    if (Input.isUltraInstinct()) {
      armControlSubsystem.setUltraInstinct(!armControlSubsystem.getUltraInstinct());
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}