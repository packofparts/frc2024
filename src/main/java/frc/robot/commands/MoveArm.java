// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawMotor;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  public double angle;
  public ArmControlSubsystem arm;
  public ClawMotor claw;
  public boolean intakeTrue;
  public MoveArm(ArmControlSubsystem armSub, ClawMotor clawSub, boolean in) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = armSub;
    claw = clawSub;
    intakeTrue = in;
    addRequirements(arm);
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveToEnum(arm.ArmSetting.GNODE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
