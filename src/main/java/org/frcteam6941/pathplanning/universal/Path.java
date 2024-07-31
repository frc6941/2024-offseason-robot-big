package org.frcteam6941.pathplanning.universal;

import java.util.ArrayList;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Translation2d;

public class Path {
    private double length;
    private ArrayList<Translation2d> pathPoints;


    public Path(double length, ArrayList<Translation2d> pathPoints) {
        this.length = length;
        this.pathPoints = pathPoints;
    }


    public double getLength() {
        return this.length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public ArrayList<Translation2d> getPathPoints() {
        return this.pathPoints;
    }

    public void setPathPoints(ArrayList<Translation2d> pathPoints) {
        this.pathPoints = pathPoints;
    }
    
    public Translation2d getPathPointByDistance(double distance) {
        Util.clamp(distance, 0.0, this.length);
        if(distance == 0.0) {
            return pathPoints.get(0);
        }
        double distanceRecord = 0.0;
        for(int i = 0; i < pathPoints.size() - 1; i++) {
            double segmentLength = pathPoints.get(i).minus(pathPoints.get(i + 1)).getNorm();
            distanceRecord += segmentLength;
            if(distanceRecord >= distance) {
                return pathPoints.get(i).interpolate(pathPoints.get(i + 1), (1 - (distanceRecord - distance) / segmentLength));
            }
        }
        return pathPoints.get(pathPoints.size() - 1);
    }
}
