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
  public boolean outtakeTrue;
  public ArmSetting armset;
  public boolean done = false;
  public MoveArm(ArmControlSubsystem armSub, ClawMotor clawSub, boolean in, boolean out, ArmSetting armsetting) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = armSub;
    claw = clawSub;
    intakeTrue = in;
    outtakeTrue = out;
    armset = armsetting;
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
<<<<<<< HEAD
    arm.moveToEnum(armset);
    if (intakeTrue) {
      claw.intake();
    }
    if (outtakeTrue) {
      claw.outtake();
    }
    done = true;
=======
    //arm.moveToEnum(arm.ArmSetting.GNODE);
>>>>>>> a34a58cc50760cf43e730524ac1113ca8541704f
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
