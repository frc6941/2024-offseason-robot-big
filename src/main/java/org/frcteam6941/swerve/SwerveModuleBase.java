package org.frcteam6941.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    int getModuleNumber();
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion);
    SwerveModuleState getState();

    double getTick();
}
