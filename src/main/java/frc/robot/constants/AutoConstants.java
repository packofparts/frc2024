package frc.robot.constants;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreCone;
import frc.robot.constants.ArmConstants.ArmState;
import frc.robot.subsystems.ArmControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class AutoConstants {

        private AutoConstants() {
                throw new IllegalStateException("Constants Class");
        }

        public static final PathConstraints DEFAULT_SPEED_CONSTRAINTS =
                        new PathConstraints(3, .75, 0.1524, 0.0381);

        // Create a list of bezier points from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use
        // holonomic rotation.
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
                        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)));


        public static final PathPlannerTrajectory STATION_2_PIECE_BLUE_PATH =
                        new PathPlannerTrajectory(PathPlannerPath.fromPathFile("Station2PieceBlue"),
                                        new ChassisSpeeds());
        public static final PathPlannerTrajectory STATION_2_PIECE_RED_PATH =
                        new PathPlannerTrajectory(PathPlannerPath.fromPathFile("Station2PieceRed"),
                                        new ChassisSpeeds());

        public static final PathPlannerTrajectory MOVE_ONE_METER = new PathPlannerTrajectory(
                        PathPlannerPath.fromPathFile("MoveOneMeter"), new ChassisSpeeds());

        public static final Map<String, Command> EMPTY_MAP = new HashMap<>();
        public static final Map<String, Command> EVENT_MAP = new HashMap<>();

        // Determines if event map should be populated
        public static final boolean DO_SUBSYSTEM_COMMANDS = true;

        // If not populated determines the wait times of former events
        public static final double TIME_BETWEEN_EVENTS_SEC = 1;

        public static void populateHashMaps(ArmControlSubsystem arm, IntakeSubsystem intake) {

                if (DO_SUBSYSTEM_COMMANDS) {
                        EVENT_MAP.put("stow_arm", new SequentialCommandGroup(
                                        arm.waitUntilSpTelescope(
                                                        ArmConstants.ArmState.STOW.extentionDistIn),
                                        arm.waitUntilSpPivot(
                                                        ArmConstants.ArmState.STOW.pivotAngleRad)));
                        EVENT_MAP.put("stow_cone", new SequentialCommandGroup(
                                        arm.waitUntilSpTelescope(
                                                        ArmConstants.ArmState.STOW.extentionDistIn),
                                        arm.waitUntilSpPivot(
                                                        ArmConstants.ArmState.STOW.pivotAngleRad)));
                        EVENT_MAP.put("ground_pickup_cone", new SequentialCommandGroup(
                                        arm.waitUntilSpPivot(
                                                        ArmConstants.ArmState.GROUND_PICKUP_CONE.pivotAngleRad),
                                        arm.waitUntilSpTelescope(
                                                        ArmConstants.ArmState.GROUND_PICKUP_CONE.extentionDistIn),
                                        new RunCommand(() -> intake.runIntake(1)).withTimeout(3)));
                        EVENT_MAP.put("score_cone_low",
                                        new ScoreCone(arm, intake, ArmState.LOWER_NODE_CONE));

                } else {
                        EVENT_MAP.put("stow_arm", new WaitCommand(TIME_BETWEEN_EVENTS_SEC));
                        EVENT_MAP.put("stow_cone", new WaitCommand(TIME_BETWEEN_EVENTS_SEC));
                        EVENT_MAP.put("ground_pickup_cone",
                                        new WaitCommand(TIME_BETWEEN_EVENTS_SEC));
                        EVENT_MAP.put("score_cone_low", new WaitCommand(TIME_BETWEEN_EVENTS_SEC));
                }
        }

}
