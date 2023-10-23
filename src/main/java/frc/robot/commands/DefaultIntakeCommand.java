// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {

  private IntakeSubsystem _intake;
  public DefaultIntakeCommand(IntakeSubsystem intake) {
    this._intake = intake;
    addRequirements(this._intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Intake Cone Outtake Cube
        if (Input.getRightTrigger() > IntakeConstants.kIntakeDeadZone) {this._intake.runIntake(Input.getRightTrigger()/1.75);}
    
        // Intake Cube Outtake Cone
        if (Input.getLeftTrigger()>IntakeConstants.kIntakeDeadZone){this._intake.runIntake(-Input.getLeftTrigger()/1.2);}
    
        //Set Intake to Stall Speed if Neither
        if (Input.getLeftTrigger()<IntakeConstants.kIntakeDeadZone && Input.getRightTrigger()<IntakeConstants.kIntakeDeadZone){this._intake.runIntake(IntakeConstants.kIntakeStallSpeed);}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
