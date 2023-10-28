// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

  private IntakeSubsystem intake;
  public DefaultIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Intake Cone Outtake Cube
        if (Input.getRightTrigger() > IntakeConstants.INPUT_DEADZONE) {this.intake.runIntake(Input.getRightTrigger()/1.75);}
    
        // Intake Cube Outtake Cone
        if (Input.getLeftTrigger()>IntakeConstants.INPUT_DEADZONE){this.intake.runIntake(-Input.getLeftTrigger()/1.2);}
    
        //Set Intake to Stall Speed if Neither
        if (Input.getLeftTrigger()<IntakeConstants.INPUT_DEADZONE && Input.getRightTrigger()<IntakeConstants.INPUT_DEADZONE){this.intake.runIntake(IntakeConstants.STALL_SPEED);}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
