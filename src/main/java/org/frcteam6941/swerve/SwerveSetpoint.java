package org.frcteam6941.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds mChassisSpeeds;
    public SwerveModuleState[] mModuleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    @Override
    public String toString() {
        StringBuilder ret = new StringBuilder(mChassisSpeeds.toString() + "\n");
        for (SwerveModuleState mModuleState : mModuleStates) {
            ret.append("  ").append(mModuleState.toString()).append("\n");
        }
        return ret.toString();
    }
}
