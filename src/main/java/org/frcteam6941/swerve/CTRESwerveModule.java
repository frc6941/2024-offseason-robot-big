package org.frcteam6941.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTRESwerveModule implements SwerveModuleBase {
	public CTRESwerveModule(int id, SwerveModuleConstants constants, String canbusName) {
        moduleNumber = id;
        module = new SwerveModule(constants, canbusName);
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public SwerveModuleState getState() {
        return module.getCurrentState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return module.getCachedPosition();
    }

    @Override
    public void updateSignals() {
        // module.getPosition(true);
        SwerveModulePosition pos = module.getPosition(true);
 	}

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
		module.apply(desiredState, isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity);
		// System.out.println(moduleNumber + " = " + desiredState.speedMetersPerSecond + " = "
		// 		+ module.getDriveMotor().getMotorVoltage() + " " + module.getSteerMotor().getMotorVoltage());
    }


    private int moduleNumber;
    private SwerveModule module;
}
