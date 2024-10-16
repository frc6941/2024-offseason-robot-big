package org.frcteam6941.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SimSwerveModuleDummy implements SwerveModuleBase {
    private final int moduleNumber;
    private final SwerveModulePosition currentPosition;
    private SwerveModuleState currentState;

    public SimSwerveModuleDummy(int id, SwerveModuleConstants constants) {
        moduleNumber = id;
        currentState = new SwerveModuleState();
        currentPosition = new SwerveModulePosition();
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public void updateSignals() {
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        currentState = desiredState;
        currentPosition.distanceMeters += desiredState.speedMetersPerSecond * Constants.LOOPER_DT * 0.95;
        currentPosition.angle = desiredState.angle;
    }

    @Override
    public SwerveModuleState getState() {
        return currentState;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return currentPosition;
    }
}
