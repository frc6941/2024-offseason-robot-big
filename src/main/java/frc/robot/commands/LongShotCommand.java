package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
//import frc.robot.drivers.LimelightHelpers;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Utils;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.Logger.debug;
import static frc.robot.Constants.SwerveDrivetrain.*;

import org.littletonrobotics.conduit.ConduitApi;

public class LongShotCommand extends Command {
	private final ShooterSubsystem shooterSubsystem;
	private final IndicatorSubsystem indicatorSubsystem;
	private final Swerve Swerve;
	private final CommandXboxController driverController;
	// private final Translation2d targetTranslation = new Translation2d(2, 2); //
	// modify this

	public LongShotCommand(
			ShooterSubsystem shooterSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			Swerve Swerve,
			CommandXboxController driverController) {
		this.shooterSubsystem = shooterSubsystem;
		this.indicatorSubsystem = indicatorSubsystem;
		this.Swerve = Swerve;
		this.driverController = driverController;
		// drive.HeadingController.enableContinuousInput(0.0, 360.0);

	}

	@Override
	public void initialize() {
		Swerve.setLockHeading(true);
	}

	@Override
	public void execute() {
		this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
		double angle = 0;
		switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
			case Blue:
				angle = -1 * Constants.SwerveDrivetrain.LongShotAngle.get();
				break;
			case Red:
				angle = Constants.SwerveDrivetrain.LongShotAngle.get();
				break;
			default:
				break;
		}

		Rotation2d targetAngle = Rotation2d.fromDegrees(angle); // test this first
		// Translation2d currentTranslation =
		// Swerve.getPose().getTranslation();
		// Translation2d deltaTranslation =
		// targetTranslation.minus(currentTranslation);.lk
		// Rotation2d targetAngle = deltaTranslation.getAngle();

		// Swerve.applyRequest(() -> drive
		// .withVelocityX(Utils.sign(-driverController.getLeftY())
		// * xLimiter.calculate(Math.abs(driverController.getLeftY()))
		// * maxSpeed.magnitude())
		// .withVelocityY(
		// Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
		// * yLimiter.calculate(Math.abs(driverController.getLeftX())))
		// .withCurrentAngle(Swerve.getState().Pose.getRotation().minus(
		// Swerve.getOffset()))
		// .withTargetAngle(targetAngle))
		// .execute();
		Swerve.drive(
				new Translation2d(
						-driverController.getLeftX() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
						-driverController.getLeftY() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
				0,
				true,
				false);
		Swerve.setHeadingTarget(Swerve.getGyro().getYaw().getDegrees() - targetAngle.getDegrees());
	}

	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.getIo().setArmPosition(Radians.zero());
	}
}
