package org.frcteam6941.utils;


import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;



/**
 * Helper class for storing and calculating a moving average of the pose2d class
 */
public class MovingAverageTransform2d {
    CopyOnWriteArrayList<Transform2d> transforms = new CopyOnWriteArrayList<Transform2d>();
    private int maxSize;

    public MovingAverageTransform2d(int maxSize) {
        this.maxSize = maxSize;
    }

    public synchronized void add(Transform2d twist) {
        transforms.add(twist);
        if (transforms.size() > maxSize) {
            transforms.remove(0);
        }
    }

    public synchronized Transform2d getAverage() {
        double x = 0.0, y = 0.0, t = 0.0;

        for (Transform2d transform : transforms) {
            x += transform.getX();
            y += transform.getY();
            t += transform.getRotation().getDegrees();
        }

        double size = getSize();
        return new Transform2d(new Translation2d(x / size, y / size), Rotation2d.fromDegrees(t / size));
    }

    public int getSize() {
        return transforms.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        transforms.clear();
    }

}