package frc.robot.utils.ShootingParameters;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Data;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class ShootingParameters {
    private double distance;
    private double shootingVelocity;
    private double shootingAngle;
    private Rotation2d fieldAimignAngle;

    public ShootingParameters(double distance,
            double shootingVelocity,
            double shootingAngle,
            Rotation2d fieldAimingAngle) {
        this.distance = distance;
        this.shootingVelocity = shootingVelocity;
        this.shootingAngle = shootingAngle;
        this.fieldAimignAngle = fieldAimingAngle;
    }
}