// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreCone extends CommandBase {
  /** Creates a new ScoreConeHighNode. */

  private final SequentialCommandGroup mPath;

  public ScoreCone(ArmControlSubsystem arm, IntakeSubsystem intake, ArmState state) {
    mPath = new SequentialCommandGroup(arm.waitUntilSpPivot(state.pivotAngleRad),
        arm.waitUntilSpTelescope(state.extentionDistIn),
        intake.timedIntake(IntakeConstants.OUTTAKE_VEL_SCORE_CONE_PERCENT,
            IntakeConstants.OUTTAKE_TIME_SCORE_CONE_SEC));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mPath.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mPath.isFinished();

  }
}
