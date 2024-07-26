package org.frcteam6941.drivers;

import edu.wpi.first.math.geometry.Rotation2d;

public class DummyGyro implements Gyro {
    Rotation2d yaw = new Rotation2d();
    Rotation2d pitch = new Rotation2d();
    Rotation2d roll = new Rotation2d();
    Rotation2d previousYaw = new Rotation2d();
    Rotation2d previousPitch = new Rotation2d();
    Rotation2d previousRoll = new Rotation2d();

    double dt;

    public DummyGyro(double looperDt) {
        dt = looperDt;
    }

    @Override
    public Rotation2d getYaw() {
        return yaw;
    }

    @Override
    public Rotation2d getRoll() {
        return roll;
    }

    @Override
    public Rotation2d getPitch() {
        return pitch;
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setYaw(double angleDeg) {
        this.yaw = Rotation2d.fromDegrees(angleDeg);
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setRoll(double angleDeg) {
        this.roll = Rotation2d.fromDegrees(angleDeg);
    }

    /**
     * Sets the pitch register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setPitch(double angleDeg) {
        this.pitch = Rotation2d.fromDegrees(angleDeg);
    }

    @Override
    public double[] getRaw() {
        return new double[] {
            (yaw.getDegrees() - previousYaw.getDegrees()) / dt,
            (pitch.getDegrees() - previousPitch.getDegrees()) / dt,
            (roll.getDegrees() - previousRoll.getDegrees()) / dt
        };
    }
}
