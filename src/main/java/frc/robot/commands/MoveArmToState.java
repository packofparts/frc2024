// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.subsystems.ArmControlSubsystem;

public class MoveArmToState extends CommandBase {
  /** Creates a new PivotCmd. */
  ArmControlSubsystem arm;
  ArmConstants.ArmState desiredState;
  Command m_command;
  
  public MoveArmToState(ArmControlSubsystem arm, ArmConstants.ArmState desiredState) {
    this.arm = arm;
    this.desiredState = desiredState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command getCommandFromEnum(ArmState armm){
    switch (armm){
      case STOW:
        switch(ArmConstants.curArmState){
            case STOW:
              return new PrintCommand("Dumbass");
            case LOWER_NODE_CONE:
              return new InstantCommand(()->this.arm.setDesiredPivotRotation(armm.pivotAngleRad));
            case MID_NODE_CONE:
              return new SequentialCommandGroup(null);
            case UPPER_NODE_CONE:
              return new SequentialCommandGroup(null);
            case GROUND_PICKUP_CONE:
              return new SequentialCommandGroup(null);
            case UNDEFINED:
              return new SequentialCommandGroup(null);
        }
      case LOWER_NODE_CONE:
        switch(ArmConstants.curArmState){
            case STOW:
              return new SequentialCommandGroup(null);
            case LOWER_NODE_CONE:
              return new PrintCommand("Dumbass");
            case MID_NODE_CONE:
              return new SequentialCommandGroup(null);
            case UPPER_NODE_CONE:
              return new SequentialCommandGroup(null);
            case GROUND_PICKUP_CONE:
              return new SequentialCommandGroup(null);
            case UNDEFINED:
              return new SequentialCommandGroup(null);
        }
      case MID_NODE_CONE:
        switch(ArmConstants.curArmState){
          case STOW:
            return new SequentialCommandGroup(null);
          case LOWER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case MID_NODE_CONE:
            return new PrintCommand("Dumbass");
          case UPPER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case GROUND_PICKUP_CONE:
            return new SequentialCommandGroup(null);
          case UNDEFINED:
            return new SequentialCommandGroup(null);
      }
      case UPPER_NODE_CONE:
        switch(ArmConstants.curArmState){
          case STOW:
            return new SequentialCommandGroup(null);
          case LOWER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case MID_NODE_CONE:
            return new SequentialCommandGroup(null);
          case UPPER_NODE_CONE:
            return new PrintCommand("Dumbass");
          case GROUND_PICKUP_CONE:
            return new SequentialCommandGroup(null);
          case UNDEFINED:
            return new SequentialCommandGroup(null);
      }
      case GROUND_PICKUP_CONE:
        switch(ArmConstants.curArmState){
          case STOW:
            return new SequentialCommandGroup(null);
          case LOWER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case MID_NODE_CONE:
            return new SequentialCommandGroup(null);
          case UPPER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case GROUND_PICKUP_CONE:
            return new PrintCommand("Dumbass");
          case UNDEFINED:
            return new SequentialCommandGroup(null);
      }
      case UNDEFINED:
        switch(ArmConstants.curArmState){
          case STOW:
            return new SequentialCommandGroup(null);
          case LOWER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case MID_NODE_CONE:
            return new SequentialCommandGroup(null);
          case UPPER_NODE_CONE:
            return new SequentialCommandGroup(null);
          case GROUND_PICKUP_CONE:
            return new SequentialCommandGroup(null);
          case UNDEFINED:
            return new PrintCommand("Dumbass");
      }
      default:
        return new PrintCommand("error");
    
    } 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_command = getCommandFromEnum(this.desiredState);
    m_command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}
