package org.frcteam6941.pathplanning.astar.obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class PolygonObstacle implements Obstacle {
    private Translation2d[] cornerPoints;

    public PolygonObstacle(Translation2d... cornerPoints) {
        this.cornerPoints = cornerPoints;
    }

    @Override
    public boolean isInObstacle(double x, double y) {
        // A point is in a polygon if a line from the point to infinity crosses the
        // polygon an odd number of times
        boolean odd = false;
        for (int i = 0, j = cornerPoints.length - 1; i < cornerPoints.length; i++) {
            if ((cornerPoints[i].getY() > y) != (cornerPoints[j].getY() > y)
                    && (x < (y - cornerPoints[i].getY())
                            * (cornerPoints[j].getX() - cornerPoints[i].getX())
                            / (cornerPoints[j].getY() - cornerPoints[i].getY())
                            + cornerPoints[i].getX())) {
                odd = !odd;
            }
            j = i;
        }
        return odd;
    }
}
