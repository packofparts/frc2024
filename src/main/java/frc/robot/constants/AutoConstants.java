package frc.robot.constants;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ScoreCone;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoConstants {

    public static PathConstraints defaultSpeedConstraints = new PathConstraints(3, .75);

    public static PathPlannerTrajectory station2PieceBlue = PathPlanner.loadPath("Station2PieceBlue", defaultSpeedConstraints);
    public static PathPlannerTrajectory station2PieceRed = PathPlanner.loadPath("Station2PieceRed", defaultSpeedConstraints);

    public static PathPlannerTrajectory moveOneMeters = PathPlanner.loadPath("MoveOneMeters", defaultSpeedConstraints);

    public static HashMap<String, Command> emptyMap = new HashMap<>();
    public static HashMap<String, Command> eventMap = new HashMap<>();


    private static boolean doAction = true;

    private static double waitTime = 10;

    public static void populateHashMaps(SwerveSubsystem swerve, Limelight lime, ArmControlSubsystem arm, PoseEstimation pose, IntakeSubsystem intake){
        
        if (doAction){  

            eventMap.put("stow_arm",new SequentialCommandGroup(
                arm.waitUntilSpTelescope(ArmConstants.ArmState.STOW.extentionDistIn),
                arm.waitUntilSpPivot(ArmConstants.ArmState.STOW.pivotAngleRad)
              ));

            eventMap.put("stow_cone",new SequentialCommandGroup(
                arm.waitUntilSpTelescope(ArmConstants.ArmState.STOW.extentionDistIn),
                arm.waitUntilSpPivot(ArmConstants.ArmState.STOW.pivotAngleRad)
              ));

            eventMap.put("ground_pickup_cone", new SequentialCommandGroup(
               arm.waitUntilSpPivot(ArmConstants.ArmState.GROUND_PICKUP_CONE.pivotAngleRad),
               arm.waitUntilSpTelescope(ArmConstants.ArmState.GROUND_PICKUP_CONE.extentionDistIn),
                new RunCommand(()->intake.runIntake(1)).withTimeout(3)
            ));
            eventMap.put("score_cone_low", new ScoreCone(arm, intake, ArmState.LOWER_NODE_CONE));

        }else{
            emptyMap.put("angle_N3", new WaitCommand(waitTime));
            emptyMap.put("angle_N2", new WaitCommand(waitTime));
            eventMap.put("angle_N1", new WaitCommand(waitTime));
            eventMap.put("angle_N0", new WaitCommand(waitTime));
            eventMap.put("angle_neutral",new WaitCommand(waitTime));

            eventMap.put("telescope_N3", new WaitCommand(waitTime));
            eventMap.put("telescope_N2", new WaitCommand(waitTime));
            eventMap.put("telescope_N1", new WaitCommand(waitTime));
            eventMap.put("telescope_N0", new WaitCommand(waitTime));
            eventMap.put("telescope_neutral", new WaitCommand(waitTime));

            eventMap.put("score_cone",  new WaitCommand(waitTime));
            eventMap.put("drop_cube",  new WaitCommand(waitTime));
            eventMap.put("intake_cube",  new WaitCommand(waitTime));

            eventMap.put("auto_balance", new WaitCommand(waitTime));
            eventMap.put("align_cube",  new WaitCommand(waitTime));
            eventMap.put("align_tag", new WaitCommand(waitTime));
        }
    }

}
