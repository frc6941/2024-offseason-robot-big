package org.frcteam6941.control;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

/**
 * Drive Signal for Holonomic Drivetrains.
 *
 * @param translation   Translation with x and y component, ranging from -1 to 1.
 * @param rotation      Rotational Magnitude, ranging from -1 to 1.
 * @param fieldOriented Whether the signal is relative to the field or not.
 */
public record HolonomicDriveSignal(@Getter Translation2d translation, @Getter double rotation,
                                   @Getter boolean fieldOriented, boolean isOpenLoop) {

    public boolean isValid(double translationThreshold, double rotationThreshold) {
        return translation.getNorm() > translationThreshold && Math.abs(rotation) > rotationThreshold;
    }
}
