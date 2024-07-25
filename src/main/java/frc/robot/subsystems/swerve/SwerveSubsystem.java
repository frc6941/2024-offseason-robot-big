package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.Pigeon2;

public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {

	/** Create a drivetrain to apply the speed */
	public static SwerveDrivetrain m_drivetrain = new SwerveDrivetrain(
			Constants.SwerveDrivetrian.DrivetrainConstants,
			Constants.SwerveDrivetrian.modules);

	/** Field centric drive request */
	public static final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.SwerveDrivetrian.deadband)	// Add a 10% deadband
			.withRotationalDeadband(Constants.SwerveDrivetrian.rotationalDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	/** Robot centric drive request */
	public static final SwerveRequest.RobotCentric m_driveRequestRobotCentric = new SwerveRequest.RobotCentric()
			.withDeadband(Constants.SwerveDrivetrian.deadband)	// Add a 10% deadband
			.withRotationalDeadband(Constants.SwerveDrivetrian.rotationalDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);
		
	/** Robot centric drive request */
	public static final SwerveRequest.PointWheelsAt m_driveRequestPointWheelsAt = new SwerveRequest.PointWheelsAt()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	/** Field centric with specific angle drive request */
	public static SwerveRequest.FieldCentricFacingAngle m_driveRequestFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(Constants.SwerveDrivetrian.deadband)	// Add a 10% deadband
			.withRotationalDeadband(Constants.SwerveDrivetrian.rotationalDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	/** IMU */
	public Pigeon2 pigeon2 = m_pigeon2;
	
	public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		CommandScheduler.getInstance().registerSubsystem(this);

		//Error detection
		var response = this.getPigeon2().clearStickyFaults();
		if (response.isError())
			System.out.println("Pigeon2 failed sticky fault clearing with error" + response);
		for (int i = 0; i < 4; i++) {
			response = this.getModule(i).getCANcoder().clearStickyFaults();
			if (response.isError())
				System.out.println("Swerve CANCoder " + i + " failed sticky fault clearing with error" + response);
			response = this.getModule(i).getDriveMotor().clearStickyFaults();
			if (response.isError())
				System.out.println("Swerve Drive " + i + " failed sticky fault clearing with error" + response);
			response = this.getModule(i).getSteerMotor().clearStickyFaults();
			if (response.isError())
				System.out.println("Swerve Steer " + i + " failed sticky fault clearing with error" + response);
		}

		//Reset the pigeon to become field centric
		m_pigeon2.reset();
	}
}
