package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import us.hebi.quickbuf.UninitializedMessageException;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import frc.robot.Constants;
import frc.robot.a.Utils;

public class ControllerDriveCommand extends Command {
	
	private final Supplier<Double> VelocityX, VelocityY, RotationalRate;
	private final SwerveSubsystem swerveSubsystem;

	public ControllerDriveCommand(
			SwerveSubsystem swerveSubsystem,
			Supplier<Double> VelocityX,
			Supplier<Double> VelocityY,
			Supplier<Double> RotationalRate) {
		this.VelocityX = VelocityX;
		this.VelocityY = VelocityY;
		this.RotationalRate = RotationalRate;
		this.swerveSubsystem = swerveSubsystem;
		// System.out.println("VelocityX = " + this.VelocityX + "\n VelocityY = " + this.VelocityY + "\n RotationalRate = "
		// 		+ this.RotationalRate + "\n");
		//To prevent two commands which both requires swerveSubsystem from being scheduled simultaneously.
		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {

	}
	
	@Override
	public void execute() {
		// Slot0Configs steerGains = new Slot0Configs()
		// 		.withKP(Constants.SwerveDrivetrian.steerGainsClass.STEER_KP.get())
		// 		.withKI(Constants.SwerveDrivetrian.steerGainsClass.STEER_KI.get())
		// 		.withKD(Constants.SwerveDrivetrian.steerGainsClass.STEER_KD.get())
		// 		.withKA(Constants.SwerveDrivetrian.steerGainsClass.STEER_KA.get())
		// 		.withKV(Constants.SwerveDrivetrian.steerGainsClass.STEER_KV.get())
		// 		.withKS(Constants.SwerveDrivetrian.steerGainsClass.STEER_KS.get());
		
		// Constants.SwerveDrivetrian.FrontLeft .withSteerMotorGains(steerGains);
		// Constants.SwerveDrivetrian.FrontRight.withSteerMotorGains(steerGains);
		// Constants.SwerveDrivetrian.BackLeft  .withSteerMotorGains(steerGains);
		// Constants.SwerveDrivetrian.BackRight.withSteerMotorGains(steerGains);
		
		// Constants.SwerveDrivetrian.modules[0] = Constants.SwerveDrivetrian.FrontLeft;
		// Constants.SwerveDrivetrian.modules[1] = Constants.SwerveDrivetrian.FrontRight;
		// Constants.SwerveDrivetrian.modules[2] = Constants.SwerveDrivetrian.BackLeft;
		// Constants.SwerveDrivetrian.modules[3] = Constants.SwerveDrivetrian.BackLeft;

		//Process the inputs to prevent too high speed/change rate
		double toApplyX = VelocityX.get();
		toApplyX = Utils.sign(-toApplyX)
				* Constants.SwerveDrivetrian.xLimiter.calculate(Math.abs(toApplyX))
				* Constants.SwerveDrivetrian.maxSpeed.magnitude();
				
		double toApplyY = VelocityY.get();
		toApplyY = Utils.sign(-toApplyY) 
				* Constants.SwerveDrivetrian.yLimiter.calculate(Math.abs(toApplyY))
				* Constants.SwerveDrivetrian.maxSpeed.magnitude();

		double toApplyOmega = RotationalRate.get();	
		toApplyOmega = -toApplyOmega * Constants.SwerveDrivetrian.maxAngularRate.magnitude();
		
		// //Deadband detection
		// toApplyX = Math.sqrt(Math.pow(toApplyX, 2) + Math.pow(toApplyY, 2)) 
		// 		< Constants.SwerveDrivetrian.deadband ? 0 : toApplyX;
		// toApplyY = Math.sqrt(Math.pow(toApplyX, 2) + Math.pow(toApplyY, 2)) 
		// 		< Constants.SwerveDrivetrian.deadband ? 0 : toApplyY;
		// toApplyOmega = Math.abs(toApplyOmega) 
		// 		< Constants.SwerveDrivetrian.rotationalDeadband ? 0 : toApplyOmega;

		System.out.println( "toApplyX = " + toApplyX + "\n toApplyY = " + toApplyY + "\n toApplyOmega = " + toApplyOmega);
		
		// Apply the speed
		// SwerveSubsystem.m_drivetrain = new SwerveDrivetrain(
		// 		Constants.SwerveDrivetrian.DrivetrainConstants,
		// 		Constants.SwerveDrivetrian.modules);
		
		SwerveSubsystem.m_drivetrain.setControl(
				SwerveSubsystem.m_driveRequest
						.withVelocityX(toApplyX) 
						.withVelocityY(toApplyY) 
						.withRotationalRate(toApplyOmega));

		// SwerveSubsystem.m_drivetrain.setControl(
		// 		SwerveSubsystem.m_driveRequestPointWheelsAt
		// 				.withModuleDirection(new Rotation2d(0)));
			
		Rotation3d pigeon3D = swerveSubsystem.getRotation3d();
		SmartDashboard.putNumber("Angle", pigeon3D.getAngle());
		//SmartDashboard.putNumber("Angle", Constants.SwerveDrivetrian.FrontRight.);
		
	}
	
	@Override
	public void end(boolean interrupted) {
		
	}
	
	@Override
	public boolean isFinished() {
		return false;
	}
	
}
