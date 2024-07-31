package org.frcteam6941.pathplanning.smoothing;

import java.util.ArrayList;

import org.frcteam6941.pathplanning.universal.Path;

import edu.wpi.first.math.geometry.Translation2d;

public class FloydTrimming {
    public static Path trimPath(Path path) {
        // Handle inputs
        if (path == null) {
            return path;
        }

        ArrayList<Translation2d> pathPoints = path.getPathPoints();
        int pathLength = pathPoints.size();
        if (pathLength > 2) {
            // Calculate the vector between -3 and -2 points, finding a direction
            Translation2d vector = pathPoints.get(pathLength - 1).minus(pathPoints.get(pathLength - 2));
            Translation2d tempVector;

            for (int i = pathLength - 3; i >= 0; i--) {
                // Calculate the vector of the -4 and -3 points
                tempVector = pathPoints.get(i + 1).minus(pathPoints.get(i));
                // Caculate cross product, if zero is produced then points are on the same
                // straight line, trim it
                if ((vector.getX() * tempVector.getY() - tempVector.getX() * vector.getY()) == 0) {
                    pathPoints.remove(i + 1);
                    // Else, update the prodcut to be the next direction indicator
                } else {
                    vector = tempVector;
                }
            }
        }
        return new Path(path.getLength(), pathPoints);
    }
}
