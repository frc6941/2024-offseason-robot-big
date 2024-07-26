package org.frcteam6941.swerve;

import lombok.Data;
import lombok.NonNull;

@Data @NonNull
public class SwerveModuleConstants {
    private int moduleNumber;

    // control constants
    private int driveMotorPort;
    private int angleMotorPort;
    private double angleOffsetDegreesCCW;
    private boolean invertAngleOutput;
    private boolean invertAngleSensorPhase; 
    private boolean onCanivore;
}
