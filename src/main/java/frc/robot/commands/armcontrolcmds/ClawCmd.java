// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armcontrolcmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawPnumatic;

public class ClawCmd extends CommandBase {
  
  ClawPnumatic clawPnumatic;
  boolean pneumatics;
  double intakeSpeed;

  double time;


  Timer timer;
  boolean useTime;

  public ClawCmd(ClawPnumatic clawPnumatic, boolean pneumatics, double intakeSpeed) {
    
    this.pneumatics = pneumatics;
    this.intakeSpeed = intakeSpeed;

    
    addRequirements(clawPnumatic);
  }

  public ClawCmd(ClawPnumatic clawPnumatic, boolean pneumatics, double intakeSpeed, double time) {
    
    this.pneumatics = pneumatics;
    this.intakeSpeed = intakeSpeed;
    this.useTime = true;
    this.time = time;


    this.timer = new Timer();

    addRequirements(clawPnumatic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawPnumatic.changePneumatics(pneumatics);
    clawPnumatic.changeIntake(intakeSpeed);
    
    if(this.useTime){
      timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.useTime && timer.get() >= this.time){
      return true;
    }
    return false;
  }
}
