// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.WestCoastDrive;

public class DriveWithJoySticks extends CommandBase {

  private CommandJoystick transJoyStick;
  private CommandJoystick rotJoyStick;
  private WestCoastDrive westCoastDrive;


  /** Creates a new DriveWithJoySticks. */
  public DriveWithJoySticks(CommandJoystick tJoystick, CommandJoystick rJoystick, WestCoastDrive wCoastDrive) {
    transJoyStick = tJoystick;
    rotJoyStick = rJoystick;
    westCoastDrive = wCoastDrive;
    addRequirements(westCoastDrive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    westCoastDrive.drive(transJoyStick.getY(), rotJoyStick.getX());
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
