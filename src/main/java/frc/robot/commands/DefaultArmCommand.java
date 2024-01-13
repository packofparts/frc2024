// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.Input;

public class DefaultArmCommand extends CommandBase {

  private final ArmControlSubsystem mArmControlSubsystem;

  public DefaultArmCommand(ArmControlSubsystem armControlSubsystem) {
    mArmControlSubsystem = armControlSubsystem;
    addRequirements(mArmControlSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    handleInput();
  }

  void handleInput() {

    if (Input.getLeftStickY() > JoystickConstants.PIVOT_INPUT_DEADZONE) {
      mArmControlSubsystem.changeDesiredPivotRotation(JoystickConstants.PIVOT_INPUT_DESATURATION
          * (Input.getLeftStickY() - JoystickConstants.PIVOT_INPUT_DEADZONE));
    } else if (Input.getLeftStickY() < -JoystickConstants.PIVOT_INPUT_DEADZONE) {
      mArmControlSubsystem.changeDesiredPivotRotation(JoystickConstants.PIVOT_INPUT_DESATURATION
          * (Input.getLeftStickY() + JoystickConstants.PIVOT_INPUT_DEADZONE));
    }

    else if (Input.getA()) {
      SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(
          () -> mArmControlSubsystem.setDesiredPivotRot(ArmConstants.ArmState.STOW.pivotAngleRad),
          mArmControlSubsystem)

      );
      command.schedule();
    } else if (Input.getB()) {
      SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(
          () -> mArmControlSubsystem
              .setDesiredPivotRot(ArmConstants.ArmState.MID_NODE_CONE.pivotAngleRad),
          mArmControlSubsystem)

      );
      command.schedule();
    } else if (Input.getX()) {
      SequentialCommandGroup command = new SequentialCommandGroup(mArmControlSubsystem
          .waitUntilSpPivot(ArmConstants.ArmState.UPPER_NODE_CONE.pivotAngleRad));
      command.schedule();
    } else if (Input.getY()) {
      SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(
          () -> mArmControlSubsystem
              .setDesiredPivotRot(ArmConstants.ArmState.SUBSTATION_CONE.pivotAngleRad),
          mArmControlSubsystem));
      command.schedule();
    } else if (Input.getDPad() == Input.DPADUP) {
      SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(
          () -> mArmControlSubsystem
              .setDesiredPivotRot(ArmConstants.ArmState.GROUND_PICKUP_CONE.pivotAngleRad),
          mArmControlSubsystem));
      command.schedule();
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
