package org.frcteam6941.pathplanning.trajectory;

import java.util.ArrayList;
import java.util.stream.Collectors;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryGenerationUtils {
    public static PathPlannerTrajectory generatePathPlannerTrajectory(
            ArrayList<Translation2d> path,
            PathConstraints config) {
        if (path.size() < 2) {
            return null;
        }

        Translation2d point1 = path.remove(0);
        Translation2d point2 = path.remove(0);
        PathPlannerTrajectory finalTrajectory;

        if (path.size() > 0) {
            PathPoint[] remaningControllingPoints = path.stream().map(x -> new PathPoint(x, new Rotation2d()))
                    .collect(Collectors.toList()).toArray(new PathPoint[] {});
            finalTrajectory = PathPlanner.generatePath(
                    config,
                    false,
                    new PathPoint(point1, new Rotation2d()),
                    new PathPoint(point2, new Rotation2d()),
                    remaningControllingPoints);
        } else {
            finalTrajectory = PathPlanner.generatePath(
                    config,
                    false,
                    new PathPoint(point1, new Rotation2d()),
                    new PathPoint(point2, new Rotation2d()));
        }
        return finalTrajectory;
    }
}
