package frc.robot.utils.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Data;

@Data
public class ShootingParameters {
    private double distance;
    private double shootingVelocity;
    private double shootingAngle;
    private Rotation2d fieldAimingAngle;

    public ShootingParameters(double distance,
            double shootingVelocity,
            double shootingAngle,
            Rotation2d fieldAimingAngle) {
        this.distance = distance;
        this.shootingVelocity = shootingVelocity;
        this.shootingAngle = shootingAngle;
        this.fieldAimingAngle = fieldAimingAngle;
    }
}