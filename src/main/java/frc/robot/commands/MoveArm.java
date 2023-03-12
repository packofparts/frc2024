// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawMotor;
import frc.robot.subsystems.ArmControlSubsystem.ArmSetting;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  public double angle;
  public ArmControlSubsystem arm;
  public ClawMotor claw;
  public boolean intakeTrue;
  public boolean outtakeTrue;
  public ArmSetting armset;
  public boolean done = false;

  private double forTime = 0;
  private boolean doForTime = false;
  private boolean indefinite = false;

  Timer timer;

  public MoveArm(ArmControlSubsystem armSub, ClawMotor clawSub, boolean in, boolean out, boolean indefinite, ArmSetting armsetting) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = armSub;
    claw = clawSub;
    intakeTrue = in;
    outtakeTrue = out;
    armset = armsetting;

   


    addRequirements(arm);
    addRequirements(claw);
  }

  public MoveArm(ArmControlSubsystem armSub, ClawMotor clawSub, boolean in, boolean out, double fortime, ArmSetting armsetting) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = armSub;
    claw = clawSub;
    intakeTrue = in;
    outtakeTrue = out;
    armset = armsetting;

    this.doForTime = true;
    this.forTime = fortime;

    timer = new Timer();
    

    addRequirements(arm);
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveToEnum(armset);



    if (intakeTrue) {
      claw.intake();
    }
    if (outtakeTrue) {
      claw.outtake();
    }

    
    
    //arm.moveToEnum(arm.ArmSetting.GNODE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    // if(this.doForTime){
    //   return arm.atAngleSetpoint() && arm.atTelescopeSetpoint() && timer.get() > this.forTime;
    // } 
    // return !this.indefinite && arm.atAngleSetpoint() && arm.atTelescopeSetpoint();
    
  }
}
