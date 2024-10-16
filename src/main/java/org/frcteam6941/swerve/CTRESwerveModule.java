package org.frcteam6941.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import lombok.Getter;

public class CTRESwerveModule implements SwerveModuleBase {
    private final int moduleNumber;
    @Getter
    private final CTRESwerveIO module;

    public CTRESwerveModule(int id, SwerveModuleConstants constants, String canbusName) {
        moduleNumber = id;
        module = new CTRESwerveIO(constants, canbusName);
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
        return module.getInternalState();
    }

    @Override
    public void updateSignals() {
        // module.getPosition(true);
        SwerveModulePosition pos = module.getPosition(true);
        SmartDashboard.putNumber("Swerve/DriverMotor " + moduleNumber, module.getDriveMotor().getPosition().getValue());
    }

    // int cnt = 0;
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        module.getSteerMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(Constants.SwerveConstants.steerGainsClass.STEER_KP.get())
                .withKI(Constants.SwerveConstants.steerGainsClass.STEER_KI.get())
                .withKD(Constants.SwerveConstants.steerGainsClass.STEER_KD.get())
                .withKA(Constants.SwerveConstants.steerGainsClass.STEER_KA.get())
                .withKV(Constants.SwerveConstants.steerGainsClass.STEER_KV.get())
                .withKS(Constants.SwerveConstants.steerGainsClass.STEER_KS.get()));
        module.getDriveMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(Constants.SwerveConstants.driveGainsClass.DRIVE_KP.get())
                .withKI(Constants.SwerveConstants.driveGainsClass.DRIVE_KI.get())
                .withKD(Constants.SwerveConstants.driveGainsClass.DRIVE_KD.get())
                .withKA(Constants.SwerveConstants.driveGainsClass.DRIVE_KA.get())
                .withKV(Constants.SwerveConstants.driveGainsClass.DRIVE_KV.get())
                .withKS(Constants.SwerveConstants.driveGainsClass.DRIVE_KS.get()));

        module.apply(desiredState, isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity);
    }
}
