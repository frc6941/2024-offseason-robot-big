package org.frcteam6941.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.team254.lib.geometry.Rotation2d;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.a.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CTRESwerveModule implements SwerveModuleBase {
	// edu.wpi.first.math.geometry.Rotation2d angleLowSpeed = new edu.wpi.first.math.geometry.Rotation2d(0, 0);
	// boolean lowSpeed = false;
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

	// int cnt = 0;
    @Override
	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
		module.getSteerMotor().getConfigurator().apply(new Slot0Configs()
				.withKP(Constants.SwerveDrivetrian.steerGainsClass.STEER_KP.get())
				.withKI(Constants.SwerveDrivetrian.steerGainsClass.STEER_KI.get())
				.withKD(Constants.SwerveDrivetrian.steerGainsClass.STEER_KD.get())
				.withKA(Constants.SwerveDrivetrian.steerGainsClass.STEER_KA.get())
				.withKV(Constants.SwerveDrivetrian.steerGainsClass.STEER_KV.get())
				.withKS(Constants.SwerveDrivetrian.steerGainsClass.STEER_KS.get()));
		module.getDriveMotor().getConfigurator().apply(new Slot0Configs()
				.withKP(Constants.SwerveDrivetrian.driveGainsClass.DRIVE_KP.get())
				.withKI(Constants.SwerveDrivetrian.driveGainsClass.DRIVE_KI.get())
				.withKD(Constants.SwerveDrivetrian.driveGainsClass.DRIVE_KD.get())
				.withKA(Constants.SwerveDrivetrian.driveGainsClass.DRIVE_KA.get())
				.withKV(Constants.SwerveDrivetrian.driveGainsClass.DRIVE_KV.get())
				.withKS(Constants.SwerveDrivetrian.driveGainsClass.DRIVE_KS.get()));
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


    private int moduleNumber;
    private SwerveModule module;
}
