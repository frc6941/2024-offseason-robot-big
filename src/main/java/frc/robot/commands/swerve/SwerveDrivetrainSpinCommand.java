package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerveDrivetrainSpinCommand extends Command {
	SwerveSubsystem swerveSubsystem;
	double toApplyOmega;
	boolean finished = false;

	/**
	 * @param toApplyOmega degrees to turn in clockwise
	 */
	public SwerveDrivetrainSpinCommand(
			SwerveSubsystem swerveSubsystem,
			double toApplyOmega
	) {
		this.swerveSubsystem = swerveSubsystem;
		this.toApplyOmega = toApplyOmega;
		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void execute() {
		System.out.println("exec");
		//double robotAngle = swerveSubsystem.pigeon2.getAngle();

		//SwerveSubsystem.m_driveRequestFacingAngle.HeadingController = new PhoenixPIDController(0.04, 0, 0);

		SwerveSubsystem.m_drivetrain.setControl(
				SwerveSubsystem.m_driveRequestFacingAngle
						.withCenterOfRotation(new Translation2d(0, 0))
						.withTargetDirection(new Rotation2d(toApplyOmega)));
		
		finished = true;
	}
	
	@Override
	public boolean isFinished() {
		return finished;
	}
	
}
