package org.frcteam6941.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon implements Gyro {
    // Actual pigeon object
    private final PigeonIMU mGyro;

    // Configs
    private boolean inverted = false;
    private Rotation2d yawAdjustmentAngle = new Rotation2d();
    private Rotation2d rollAdjustmentAngle = new Rotation2d();
    private Rotation2d pitchAdjustmentAngle = new Rotation2d();

    public Pigeon(int port) {        
        mGyro = new PigeonIMU(port);
        mGyro.configFactoryDefault();
    }

    @Override
    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().minus(yawAdjustmentAngle);
        if (inverted) {
            return angle.unaryMinus();
        }
        return angle;
    }

    @Override
    public Rotation2d getRoll() {
        return getUnadjustedRoll().minus(rollAdjustmentAngle);
    }

    @Override
    public Rotation2d getPitch() {
        return getUnadjustedPitch().minus(pitchAdjustmentAngle);
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    /**
     * Sets the pitch register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setPitch(double angleDeg) {
        pitchAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    public void setInverted(boolean inv) {
        inverted = inv;
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw());
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll());
    }

    public double getYawAngularVelocity() {
        double[] xyz_dps = new double[] {0.0, 0.0, 0.0};
        mGyro.getRawGyro(xyz_dps);
        return xyz_dps[2];
    }
    @Override
    public double[] getRaw() {
        double[] xyz_dps = new double[] {0.0, 0.0, 0.0};
        mGyro.getRawGyro(xyz_dps);
        return xyz_dps;
    }
}
