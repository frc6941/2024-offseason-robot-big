package org.frcteam6941.control;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive Signal for Holonomic Drivetrains.
 * 
 * @param translation Translation with x and y component, ranging from -1 to 1.
 * @param rotation Rotational Magnitude, ranging from -1 to 1.
 * @param fieldOriented Whether the signal is relative to the field or not.
 */
public class HolonomicDriveSignal {
    private final Translation2d translation;
    private final double rotation;
    private final boolean fieldOriented;
    private final boolean isOpenLoop;

    public HolonomicDriveSignal(Translation2d translation, double rotation, boolean fieldOriented, boolean isOpenLoop) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
        this.isOpenLoop = isOpenLoop;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }

    public boolean isOpenLoop() {
        return isOpenLoop;
    }

    public boolean isValid(double translationThreshold, double rotationThreshold) {
        return translation.getNorm() > translationThreshold && Math.abs(rotation) > rotationThreshold;
    }
}
