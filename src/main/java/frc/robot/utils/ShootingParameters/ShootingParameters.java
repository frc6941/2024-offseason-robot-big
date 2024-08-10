package frc.robot.utils.ShootingParameters;

public class ShootingParameters {
    private double shootingVelocity;
    private double shootingAngle;

    public ShootingParameters(double velocity, double angle) {
        this.shootingVelocity = velocity;
        this.shootingAngle = angle;
    }

    public double getVelocity() {
        return shootingVelocity;
    }

    public double getAngle() {
        return shootingAngle;
    }

    @Override
    public String toString() {
        return String.format("<Voltage: %.2f V, Angle: %.2f rad>", shootingVelocity, shootingAngle);
    }
}