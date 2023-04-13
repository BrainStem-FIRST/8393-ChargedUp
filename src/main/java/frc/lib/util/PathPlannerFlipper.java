package frc.lib.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class PathPlannerFlipper {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
    private PathPlannerFlipper() {
        new RuntimeException("Utility Class should not be constructed");
    }

    public static PathPlannerTrajectory flipTrajectory(PathPlannerTrajectory trajectory, PathConstraints constraints) {
        List<Trajectory.State> origStates = trajectory.getStates();
        List<Trajectory.State> trajStatesNew = List.of();

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            for (Trajectory.State state : origStates) {
                trajStatesNew.add(flipState(state));
            }

            return new PathPlannerTrajectory(trajStatesNew,
                    trajectory.getMarkers(),
                    trajectory.getStartStopEvent(),
                    trajectory.getEndStopEvent(),
                    trajectory.fromGUI);
        } else {
            return trajectory;
        }
    }

    public static Trajectory.State flipState(Trajectory.State state) {
        return new Trajectory.State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(
                        FIELD_LENGTH_METERS - state.poseMeters.getX(),
                        state.poseMeters.getY(),
                        state.poseMeters.getRotation()
                ),
                state.curvatureRadPerMeter
        );
    }
}
