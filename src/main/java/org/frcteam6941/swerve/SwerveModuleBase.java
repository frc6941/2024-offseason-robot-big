package org.frcteam6941.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    int getModuleNumber();
    void updateSignals();
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion);
    SwerveModuleState getState();
    SwerveModulePosition getPosition();
}
