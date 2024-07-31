package org.frcteam6941.swerve;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class CTRESwerveModule implements SwerveModuleBase {
    private int moduleNumber;
    private CTRESwerveIO module;

    // edu.wpi.first.math.geometry.Rotation2d angleLowSpeed = new edu.wpi.first.math.geometry.Rotation2d(0, 0);
    // boolean lowSpeed = false;
    public CTRESwerveModule(int id, SwerveModuleConstants constants, String canbusName) {
        moduleNumber = id;
        module = new CTRESwerveIO(constants, canbusName);

        // FIXME: not sure if needed. if does, do change the initMotorConfigs() in CTREModuleIO.
        module.getDriveMotor().getConfigurator().apply(new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(Constants.SwerveDrivetrain.VOLTAGE_CLOSED_LOOP_RAMP_PERIOD));
        module.getDriveMotor().getConfigurator()
                .apply(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(110)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(90)
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyTimeThreshold(0.5));
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
    }

    // int cnt = 0;
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        module.getSteerMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(Constants.SwerveDrivetrain.steerGainsClass.STEER_KP.get())
                .withKI(Constants.SwerveDrivetrain.steerGainsClass.STEER_KI.get())
                .withKD(Constants.SwerveDrivetrain.steerGainsClass.STEER_KD.get())
                .withKA(Constants.SwerveDrivetrain.steerGainsClass.STEER_KA.get())
                .withKV(Constants.SwerveDrivetrain.steerGainsClass.STEER_KV.get())
                .withKS(Constants.SwerveDrivetrain.steerGainsClass.STEER_KS.get()));
        module.getDriveMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(Constants.SwerveDrivetrain.driveGainsClass.DRIVE_KP.get())
                .withKI(Constants.SwerveDrivetrain.driveGainsClass.DRIVE_KI.get())
                .withKD(Constants.SwerveDrivetrain.driveGainsClass.DRIVE_KD.get())
                .withKA(Constants.SwerveDrivetrain.driveGainsClass.DRIVE_KA.get())
                .withKV(Constants.SwerveDrivetrain.driveGainsClass.DRIVE_KV.get())
                .withKS(Constants.SwerveDrivetrain.driveGainsClass.DRIVE_KS.get()));
        // if (Math.abs(desiredState.speedMetersPerSecond) <= 0.15 * Constants.SwerveDrivetrian.maxSpeed.magnitude()) {
        // 	if (!lowSpeed) {
        // 		angleLowSpeed = desiredState.angle;
        // 		lowSpeed = true;
        // 	}
        // 	desiredState.angle = angleLowSpeed;
        // }
        // else if (lowSpeed) {
        // 	lowSpeed = false;
        // }

        module.apply(desiredState, isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity);
        // System.out.println(moduleNumber + " = " + desiredState.speedMetersPerSecond + " = "
        // 		+ module.getDriveMotor().getMotorVoltage() + " " + module.getSteerMotor().getMotorVoltage());//speed output
		/*
		SmartDashboard.putNumber("Speed m/s", desiredState.speedMetersPerSecond);
		SmartDashboard.putString("Drive Motor Voltage", module.getDriveMotor().getMotorVoltage());
		SmartDashboard.putNumber("Steer Motor Voltage", module.getSteerMotor().getMotorVoltage());
		*/
        // cnt++;
        // if (cnt % 50 == 0) {
        // 	System.out.println(desiredState.speedMetersPerSecond);
        // }
    }
}
