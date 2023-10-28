package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreCone;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class AutoConstants {

    private AutoConstants(){
        throw new IllegalStateException("Constants Class");
    }

    public static final PathConstraints kDefaultSpeedConstraints = new PathConstraints(3, .75);

    public static final PathPlannerTrajectory kStation2PieceBlue = PathPlanner.loadPath("Station2PieceBlue", kDefaultSpeedConstraints);
    public static final PathPlannerTrajectory kStation2PieceRed = PathPlanner.loadPath("Station2PieceRed", kDefaultSpeedConstraints);

    public static final PathPlannerTrajectory kMoveOneMeter = PathPlanner.loadPath("MoveOneMeters", kDefaultSpeedConstraints);

    public static Map<String, Command> emptyMap = new HashMap<>();
    public static Map<String, Command> eventMap = new HashMap<>();

    //Determines if event map should be populated
    public static final boolean DO_ACTION = true;

    //If not populated determines the wait times of former events
    public static final double WAIT_TIME = 1;

    public static void populateHashMaps(ArmControlSubsystem arm, IntakeSubsystem intake){
        
        if (DO_ACTION){  
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
            emptyMap.put("angle_N3", new WaitCommand(WAIT_TIME));
            emptyMap.put("angle_N2", new WaitCommand(WAIT_TIME));
            eventMap.put("angle_N1", new WaitCommand(WAIT_TIME));
            eventMap.put("angle_N0", new WaitCommand(WAIT_TIME));
            eventMap.put("angle_neutral",new WaitCommand(WAIT_TIME));

            eventMap.put("telescope_N3", new WaitCommand(WAIT_TIME));
            eventMap.put("telescope_N2", new WaitCommand(WAIT_TIME));
            eventMap.put("telescope_N1", new WaitCommand(WAIT_TIME));
            eventMap.put("telescope_N0", new WaitCommand(WAIT_TIME));
            eventMap.put("telescope_neutral", new WaitCommand(WAIT_TIME));

            eventMap.put("score_cone",  new WaitCommand(WAIT_TIME));
            eventMap.put("drop_cube",  new WaitCommand(WAIT_TIME));
            eventMap.put("intake_cube",  new WaitCommand(WAIT_TIME));

            eventMap.put("auto_balance", new WaitCommand(WAIT_TIME));
            eventMap.put("align_cube",  new WaitCommand(WAIT_TIME));
            eventMap.put("align_tag", new WaitCommand(WAIT_TIME));
        }
    }

}
