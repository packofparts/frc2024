// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreCone extends CommandBase {
  /** Creates a new ScoreConeHighNode. */

  private SequentialCommandGroup path;
  
  public ScoreCone(ArmControlSubsystem arm, IntakeSubsystem intake, ArmState state) {
    path = new SequentialCommandGroup(
      arm.waitUntilSpPivot(state.pivotAngleRad),
      arm.waitUntilSpTelescope(state.extentionDistIn),
      new RunCommand(() -> intake.runIntake(-1)).withTimeout(3),
      new InstantCommand(()-> intake.runIntake(0))
      );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return path.isFinished();
    
  }
}