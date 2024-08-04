package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingParameters;
import frc.robot.utils.ShootingParametersTable;

public class SpeakerAimingCommand extends Command {
	private final ShooterSubsystem shooterSubsystem;
	private final IndicatorSubsystem indicatorSubsystem;
	private final Swerve Swerve;
	private final CommandXboxController driverController;
	private Measure<Angle> defaultAngle = Degrees.of(20);
	LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

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
		Swerve.setLockHeading(true);
	}

	@Override
	public void execute() {
		this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
		var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();

		//Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
		var distance = Limelight.getInstance().getSpeakerRelativePosition().getNorm();
		//var angle = -Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees();

		ShootingParameters parameter = ShootingParametersTable.getInstance().getParameters(distance);

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
		double A1 = 18.43145;
		double A2 = 67.62172;
		double x0 = 2.07751;
		double p = 5.16297;
		offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p));
		// double A1 = 69.6287;
		// double A2 = 11.4576;
		// double x0 = 1.99428;
		// double p = -4.33742;
		// offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p)) + 1. ;

		// if (Math.abs(
		// 		offset -
		// 				shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 0.5) {
		// 	defaultAngle = Degrees.of(offset);
		// 	shooterSubsystem
		// 			.getIo()
		// 			.setArmPosition(
		// 					Radians.of(
		// 							Units.degreesToRadians(parameter.getAngle())));
		// }
		Swerve.drive(
				new Translation2d(
						-driverController.getLeftX() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
						-driverController.getLeftY() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
				0,
				true,
				false);
		filter.calculate(Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees());
		Swerve.setHeadingTarget(filter.lastValue());//.getRotation().getDegrees()
	}

	@Override
	public void end(boolean interrupted) {
		Swerve.setLockHeading(false);
		shooterSubsystem.getIo().setArmPosition(Radians.zero());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
