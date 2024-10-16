package org.frcteam6941.drivers;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Setter;

public class Pigeon2Gyro implements Gyro {
    // Actual pigeon object
    public final Pigeon2 mGyro;

    // Configs
    @Setter
    private boolean inverted = false;
    private Rotation2d yawAdjustmentAngle = new Rotation2d(0);
    private Rotation2d rollAdjustmentAngle = new Rotation2d();
    private Rotation2d pitchAdjustmentAngle = new Rotation2d();

    public Pigeon2Gyro(int port) {
        mGyro = new Pigeon2(port, "");
    }

    public Pigeon2Gyro(int port, String canbus) {
        mGyro = new Pigeon2(port, canbus);
    }

    @Override
    public Rotation2d getYaw() {
        double angle = getUnadjustedYaw().getDegrees();
        angle %= 360;
        angle = angle > 180 ? angle - 360 : angle;
        angle = angle < -180 ? angle + 360 : angle;
        angle -= yawAdjustmentAngle.getDegrees();
        angle %= 360;
        angle = angle > 180 ? angle - 360 : angle;
        angle = angle < -180 ? angle + 360 : angle;
        if (inverted) {
            return new Rotation2d(Math.toRadians(-angle));
        }
        return new Rotation2d(Math.toRadians(angle));
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setYaw(double angleDeg) {
        //yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
        yawAdjustmentAngle = Rotation2d.fromDegrees(yawAdjustmentAngle.getDegrees() + angleDeg);
        //System.out.println(yawAdjustmentAngle);
    }

    @Override
    public Rotation2d getRoll() {
        return getUnadjustedRoll().minus(rollAdjustmentAngle);
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

    @Override
    public Rotation2d getPitch() {
        return getUnadjustedPitch().minus(pitchAdjustmentAngle);
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

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble());
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch().getValueAsDouble());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll().getValueAsDouble());
    }

    public double getYawAngularVelocity() {
        return mGyro.getAngularVelocityYWorld().getValueAsDouble();
    }

    @Override
    public double[] getRaw() {
        // mGyro.getRawGyro(xyz_dps);
        return new double[]{0.0, 0.0, 0.0};
    }
}
