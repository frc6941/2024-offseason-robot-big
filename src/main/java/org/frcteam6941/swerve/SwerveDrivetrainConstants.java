package org.frcteam6941.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import lombok.Data;
import lombok.NonNull;

@Data @NonNull
public class SwerveDrivetrainConstants {
    // mechanical constants
    private DCMotor driveMotor;
    private DCMotor angleMotor;
    private double freeSpeedMetersPerSecond;
    private double maxAccelerationMetersPerSecondSquare;
    private double wheelCircumferenceMeters;
    private double driveGearRatio;
    private double angleGearRatio;
    private double deadband;
    private double drivetrainWidthBumped;
    private double drivetrainWidthFrame;
    private Translation2d drivetrainCenterofRotation;
    private Translation2d[] drivetrainModPositions;
    private SwerveDriveKinematics drivetrainKinematics;

    // overall control constants
    private double driveKP;
    private double driveKI;
    private double driveKD;
    private double driveKV;
    private double angleKP;
    private double angleKI;
    private double angleKD;
    private double angleKV;
}
