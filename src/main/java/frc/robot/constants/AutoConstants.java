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

    public static final PathConstraints DEFAULT_SPEED_CONSTRAINTS = new PathConstraints(3, .75);

    public static final PathPlannerTrajectory STATION_2_PIECE_BLUE_PATH = PathPlanner.loadPath("Station2PieceBlue", DEFAULT_SPEED_CONSTRAINTS);
    public static final PathPlannerTrajectory STATION_2_PIECE_RED_PATH = PathPlanner.loadPath("Station2PieceRed", DEFAULT_SPEED_CONSTRAINTS);

    public static final PathPlannerTrajectory MOVE_ONE_METER = PathPlanner.loadPath("MoveOneMeter", DEFAULT_SPEED_CONSTRAINTS);

    public static final Map<String, Command> EMPTY_MAP = new HashMap<>();
    public static final Map<String, Command> EVENT_MAP = new HashMap<>();

    //Determines if event map should be populated
    public static final boolean DO_ACTION = true;

    //If not populated determines the wait times of former events
    public static final double WAIT_TIME = 1;

    public static void populateHashMaps(ArmControlSubsystem arm, IntakeSubsystem intake){
        
        if (DO_ACTION){  
            EVENT_MAP.put("stow_arm",new SequentialCommandGroup(
                arm.waitUntilSpTelescope(ArmConstants.ArmState.STOW.extentionDistIn),
                arm.waitUntilSpPivot(ArmConstants.ArmState.STOW.pivotAngleRad)
              ));
            EVENT_MAP.put("stow_cone",new SequentialCommandGroup(
                arm.waitUntilSpTelescope(ArmConstants.ArmState.STOW.extentionDistIn),
                arm.waitUntilSpPivot(ArmConstants.ArmState.STOW.pivotAngleRad)
              ));
            EVENT_MAP.put("ground_pickup_cone", new SequentialCommandGroup(
               arm.waitUntilSpPivot(ArmConstants.ArmState.GROUND_PICKUP_CONE.pivotAngleRad),
               arm.waitUntilSpTelescope(ArmConstants.ArmState.GROUND_PICKUP_CONE.extentionDistIn),
                new RunCommand(()->intake.runIntake(1)).withTimeout(3)
            ));
            EVENT_MAP.put("score_cone_low", new ScoreCone(arm, intake, ArmState.LOWER_NODE_CONE));

        }else{
            EVENT_MAP.put("stow_arm",new WaitCommand(WAIT_TIME));
            EVENT_MAP.put("stow_cone",new WaitCommand(WAIT_TIME));
            EVENT_MAP.put("ground_pickup_cone", new WaitCommand(WAIT_TIME));
            EVENT_MAP.put("score_cone_low", new WaitCommand(WAIT_TIME));
        }
    }

}
