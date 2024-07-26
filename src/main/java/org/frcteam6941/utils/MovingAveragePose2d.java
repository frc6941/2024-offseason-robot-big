package org.frcteam6941.utils;


import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Helper class for storing and calculating a moving average of the pose2d class
 */
public class MovingAveragePose2d {
    CopyOnWriteArrayList<Pose2d> poses = new CopyOnWriteArrayList<>();
    private int maxSize;

    public MovingAveragePose2d(int maxSize) {
        this.maxSize = maxSize;
    }

    public synchronized void add(Pose2d pose) {
        poses.add(pose);
        if (poses.size() > maxSize) {
            poses.remove(0);
        }
    }

    public synchronized Pose2d getAverage() {
        double x = 0.0, y = 0.0, t = 0.0;

        for (Pose2d pose : poses) {
            x += pose.getX();
            y += pose.getY();
            t += pose.getRotation().getDegrees();
        }

        double size = getSize();
        return new Pose2d(x / size, y / size, Rotation2d.fromDegrees(t / size));
    }

    public int getSize() {
        return poses.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        poses.clear();
    }

}