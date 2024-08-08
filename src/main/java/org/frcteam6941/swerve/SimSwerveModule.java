package org.frcteam6941.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimSwerveModule implements SwerveModuleBase {
    private final int moduleNumber;
    private final SimSwerveIO module;

    public SimSwerveModule(int id, SwerveModuleConstants constants, String unused) {
        moduleNumber = id;
        module = new SimSwerveIO(constants);
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public void updateSignals() {
        // FIXME: will it work properly?
        module.update(0.02, 12.0);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        // FIXME: Getting zero velocity from desiredState
        module.apply(desiredState, isOpenLoop ? SwerveModule.DriveRequestType.OpenLoopVoltage : SwerveModule.DriveRequestType.Velocity);
    }

    @Override
    public SwerveModuleState getState() {
        return module.getCurrentState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return module.getInternalState();
    }
}
