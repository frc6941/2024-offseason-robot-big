package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Degrees;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;

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
		Swerve.setLockHeading(true);
	}

	@Override
	public void execute() {

		this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
		Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
		var distance = target.getTranslation().getDistance(new Translation3d());
		var angle = Units.radiansToDegrees(target.getRotation().getAngle());

		this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

		// Swerve.setLockHeading(LimelightHelpers.getTV("limelight"));
		// Swerve.drive(null, DRIVE_GEAR_RATIO, hasTarget, hasTarget);
		Swerve.drive(
				new Translation2d(
						-driverController.getLeftX() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
						-driverController.getLeftY() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
				0,
				true,
				false);
		Swerve.setHeadingTarget(-Limelight.getInstance().getSpeakerRelativePosition());//.getRotation().getDegrees()
	}

	@Override
	public void end(boolean interrupted) {
		Swerve.setLockHeading(false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
