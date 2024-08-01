package frc.robot.commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingParameters;
import frc.robot.utils.ShootingParametersTable;
import frc.robot.utils.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.Logger.debug;
import static frc.robot.Constants.SwerveDrivetrain.*;

public class SpeakerAimingCommand extends Command {
	private final ShooterSubsystem shooterSubsystem;
	private final IndicatorSubsystem indicatorSubsystem;
	private final Swerve Swerve;
	private final CommandXboxController driverController;
	private Measure<Angle> defaultAngle = Degrees.of(20);

	public SpeakerAimingCommand(
			ShooterSubsystem shooterSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			Swerve Swerve,
			CommandXboxController driverController) {
		this.shooterSubsystem = shooterSubsystem;
		this.indicatorSubsystem = indicatorSubsystem;
		this.Swerve = Swerve;
		this.driverController = driverController;
		// drive.HeadingController.s;
		// drive.HeadingController.set
	}

	@Override
	public void initialize() {
		defaultAngle = Degrees.of(20);
	}

	@Override
	public void execute() {

		this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
		var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
		boolean hasTarget = LimelightHelpers.getTV("limelight");

		if (!hasTarget) {
			debug(" " + hasTarget);
			return;
		}
		Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
		var distance = target.getTranslation().getDistance(new Translation3d());
		var angle = Units.radiansToDegrees(target.getRotation().getAngle());
		// if (distance == 0) {
		// debug("Shooter:", "wtf?");
		// shooterSubsystem.getIo().setArmPosition(defaultAngle);
		// return;
		// }

		ShootingParameters parameter = ShootingParametersTable.getInstance().getParameters(distance);
		debug("Shooter:",
				" distance => " + distance + " angle => " + parameter.getAngle());
		this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

		// Calculated using highly-sophisticated software.
		// Do not touch unless you (really) know what you're doing!
		// double A = -317.1;
		// double B = 631.7;
		// double C = -489.2;
		// double D = 200.1;
		// double E = -40.88;
		// double F = 3.268;
		// offset = A + B * distance + C * Math.pow(distance, 2) + D *
		// Math.pow(distance, 3) + E * Math.pow(distance, 4) + F * Math.pow(distance,
		// 5);
		// double A1 = 18.43145;
		// double A2 = 67.62172;
		// double x0 = 2.07751;
		// double p = 5.16297;
		// offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p));
		// double A1 = 69.6287;
		// double A2 = 11.4576;
		// double x0 = 1.99428;
		// double p = -4.33742;
		// offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p)) + 1. ;

		// debug("Shooter:", "desired angle = " + offset);
		// //debug("Shooter:", "actual angle = " +
		// shooterSubsystem.getInputs().armPosition.in(Degrees));
		SmartDashboard.putNumber("shooter desired angle", Units.degreesToRadians(parameter.getAngle()));
		SmartDashboard.putNumber("heading angle", Swerve.getGyro().getYaw().getDegrees());
		SmartDashboard.putNumber("tag angle",
				Swerve.getGyro().getYaw().getDegrees() + LimelightHelpers.getTX("limelight"));

		if (0 > offset || offset > 180) {
			debug("Shooter:", "wtf?");
			shooterSubsystem.getIo().setArmPosition(defaultAngle);
			return;
		}

		if (Math.abs(
				offset -
						shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 0.5) {
			defaultAngle = Degrees.of(offset);
			shooterSubsystem
					.getIo()
					.setArmPosition(
							Radians.of(
									Units.degreesToRadians(parameter.getAngle())));
		}
		Swerve.setLockHeading(LimelightHelpers.getTV("limelight"));
		// Swerve.drive(null, DRIVE_GEAR_RATIO, hasTarget, hasTarget);
		Swerve.drive(
				new Translation2d(
						-driverController.getLeftX() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
						-driverController.getLeftY() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
				LimelightHelpers.getTV("limelight") ? 0
						: 1 * driverController.getRightX()
								* Constants.SwerveDrivetrain.maxAngularRate.magnitude(),
				true,
				false);
		Swerve.setHeadingTarget(Swerve.getGyro().getYaw().getDegrees() - LimelightHelpers.getTX("limelight"));
		// Swerve.applyRequest(() -> drive
		// .withVelocityX(Utils.sign(-driverController.getLeftY())
		// * xLimiter.calculate(Math.abs(driverController.getLeftY()))
		// * maxSpeed.magnitude())
		// .withVelocityY(
		// Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
		// * yLimiter.calculate(Math.abs(driverController.getLeftX())))
		// .withCurrentTx(LimelightHelpers.getTX("limelight") * 1.6)).execute();

		// debug(" " + Constants.HeadingController.HEADING_KP.get());
	}

	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.getIo().setArmPosition(Radians.zero());
		Swerve.setLockHeading(false);
	}
}
