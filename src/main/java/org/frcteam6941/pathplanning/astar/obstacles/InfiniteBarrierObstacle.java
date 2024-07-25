package org.frcteam6941.pathplanning.astar.obstacles;

public class InfiniteBarrierObstacle implements Obstacle{
    public enum DIRECTION {
        GREATER, SMALLER
    }

    public enum AXIS {
        X, Y
    }

    public double barrier;
    public DIRECTION direction;
    public AXIS axis;


    public InfiniteBarrierObstacle(double barrier, DIRECTION direction, AXIS axis) {
        this.barrier = barrier;
        this.direction = direction;
        this.axis = axis;
    }

    @Override
    public boolean isInObstacle(double x, double y) {
        switch(direction) {
            case GREATER:
                switch(axis) {
                    case X:
                        return x > barrier;
                    case Y:
                        return y > barrier;
                    default:
                        return false;
                }
            case SMALLER:
                switch(axis) {
                    case X:
                        return x < barrier;
                    case Y:
                        return y < barrier;
                    default:
                        return false;
                }
            default:
                return false;
        }
    }
}
