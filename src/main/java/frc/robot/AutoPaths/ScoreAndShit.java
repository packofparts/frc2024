// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoPaths;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoMapConstants.GamePiece;
import frc.robot.commands.armcontrolcmds.ExtensionCmd;
import frc.robot.commands.armcontrolcmds.PivotCmd;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.ClawPnumatic;

public class ScoreAndShit extends CommandBase {
  /** Creates a new ScoreConeHighNode. */
  ArmControlSubsystem arm;
  ClawPnumatic claw;
  SequentialCommandGroup path;
  
  public ScoreAndShit(ArmControlSubsystem arm, ClawPnumatic claw) {
    this.arm = arm;
    this.claw = claw;
    addRequirements(this.arm,this.claw);
    path = new SequentialCommandGroup(
      new PivotCmd(this.arm, ArmConstants.angleLevelsRad[2]),
      new ExtensionCmd(this.arm, ArmConstants.extensionLevelsIn[2]),
      new WaitCommand(.4),
      claw.dropPiece(GamePiece.CONE),
      new ExtensionCmd(this.arm, ArmConstants.extensionLevelsIn[0]),
      new PivotCmd(this.arm, ArmConstants.angleLevelsRad[0])
      
      );
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    path.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return path.isFinished();
    
  }
}
