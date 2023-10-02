// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Joysticks;

public class DriveCommand extends CommandBase {
  
  DriveSubsystem swervee;
  Joysticks joyee;

  public DriveCommand(Joysticks joys, DriveSubsystem system) {
    this.joyee = joys;
    this.swervee = system;
    
    addRequirements(system);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    double x = this.joyee.getX();
    double y = this.joyee.getY();
    double rot = this.joyee.getRot();

    this.swervee.setMotors(x, y, rot);

  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
