package org.frcteam6941.utils;

public final class AngleNormalization {
    public static double getAbsoluteAngleRadian(double position) {
        double angle = Math.toRadians(position);
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public static double getAbsoluteAngleDegree(double position) {
        double angle = position;
        angle %= 360.0;
        if (angle < 0.0) {
            angle += 360.0;
        }
        return angle;
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    	double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
        	lowerBound = scopeReference - lowerOffset;
        	upperBound = scopeReference + (360 - lowerOffset);
        }else{
        	upperBound = scopeReference - lowerOffset; 
        	lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound) {
        	newAngle += 360; 
        }
        while(newAngle > upperBound) {
        	newAngle -= 360; 
        }
        if (newAngle - scopeReference > 180) {
        	newAngle -= 360;
        }else if (newAngle - scopeReference < -180) {
        	newAngle += 360;
        }
        return newAngle;
    }
}
