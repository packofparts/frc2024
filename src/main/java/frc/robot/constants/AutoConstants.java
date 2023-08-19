package frc.robot.constants;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoConstants {

    public static PathConstraints defaultSpeedConstraints = new PathConstraints(4, 2);

    public static PathPlannerTrajectory station2Piece = PathPlanner.loadPath("Station2Piece", defaultSpeedConstraints);

    public static HashMap<String, Command> emptyMap = new HashMap<>();

}
