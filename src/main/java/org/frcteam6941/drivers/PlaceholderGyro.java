package org.frcteam6941.drivers;

import edu.wpi.first.math.geometry.Rotation2d;

public class PlaceholderGyro implements Gyro{
    @Override
    public Rotation2d getYaw() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getPitch() {
        return new Rotation2d();
    }

    @Override
    public Rotation2d getRoll() {
        return new Rotation2d();
    }

    @Override
    public void setYaw(double angle) {

    }

    @Override
    public void setPitch(double angle) {
        
    }

    @Override
    public void setRoll(double angle) {
        
    }

    @Override
    public double[] getRaw() {
        return new double[] { 0.0, 0.0, 0.0 };
    }
}
