package org.frcteam6941.pathplanning.astar.obstacles;

import edu.wpi.first.math.geometry.Translation2d;

public class RectangularObstacle implements Obstacle{
    private double minX = Double.POSITIVE_INFINITY;
    private double maxX = Double.NEGATIVE_INFINITY;
    private double minY = Double.POSITIVE_INFINITY;
    private double maxY = Double.NEGATIVE_INFINITY;


    public RectangularObstacle(Translation2d... cornerPoints) {
        for(Translation2d corner:cornerPoints) {
            if(corner.getX() < minX) {
                minX = corner.getX();
            }
            if(corner.getX() > maxX) {
                maxX = corner.getX();
            }
            if(corner.getY() < minY) {
                minY = corner.getY();
            }
            if(corner.getY() > maxY) {
                maxY = corner.getY();
            }
        }
    }

    public RectangularObstacle(Translation2d[] cornerPoints, double marginx, double marginy) {
        this(cornerPoints);
        marginx = Math.abs(marginx);
        marginy = Math.abs(marginy);
        minX -= marginx;
        minY -= marginy;
        maxX += marginx;
        maxY += marginy;
    }


    @Override
    public boolean isInObstacle(double x, double y) {
        return x > minX && x < maxX && y > minY && y < maxY;
    }
}
