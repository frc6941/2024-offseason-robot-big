package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.Measure;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Degrees;

public class SpeakerAimingCommand extends Command {

	private Measure<Angle> defaultAngle=Degrees.of(20);
	
	private final Supplier<Double> VelocityX, VelocityY, RotationalRate;
	private final Swerve swerve;
	ShooterIOTalonFX shooterSubsystem;

	public SpeakerAimingCommand(
			Swerve swerve,
			Supplier<Double> VelocityX,
			Supplier<Double> VelocityY,
			Supplier<Double> RotationalRate,
			ShooterIOTalonFX shooterSubsystem) {
		this.VelocityX = VelocityX;
		this.VelocityY = VelocityY;
		this.RotationalRate = RotationalRate;
		this.swerve = swerve;
		this.shooterSubsystem = shooterSubsystem;
		// System.out.println("VelocityX = " + this.VelocityX + "\n VelocityY = " + this.VelocityY + "\n RotationalRate = "
		// 		+ this.RotationalRate + "\n");
		//To prevent two commands which both requires swerveSubsystem from being scheduled simultaneously.
		addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
		//swerve.setLockHeading(true);
		//swerve.resetHeadingController();
	}
	
	@Override
	public void execute() {
		swerve.setLockHeading(LimelightHelpers.getTV("limelight"));
		swerve.drive(
				new Translation2d(
						VelocityX.get() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
						VelocityY.get() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
						LimelightHelpers.getTV("limelight")?0:1 * RotationalRate.get() * Constants.SwerveDrivetrain.maxAngularRate.magnitude(),
								true,
				false);
		
		//this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        if (!hasTarget) {
            shooterSubsystem.setArmPosition(defaultAngle);
            return;
        }
        Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
        var distance = target.getTranslation().getDistance(new Translation3d());
        var angle = Units.radiansToDegrees(target.getRotation().getAngle());
        // if (distance == 0) {
        //     debug("Shooter:", "wtf?");
        //     shooterSubsystem.getIo().setArmPosition(defaultAngle);
        //     return;
        // }
        // debug("Shooter:",
        //         " distance => " + distance + " angle => " + angle);
        // this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

        // Calculated using highly-sophisticated software.
        // Do not touch unless you (really) know what you're doing!
//        offset = -297 + 573.9 * distance - 427.3 * Math.pow(distance, 2) + 168.9 * Math.pow(distance, 3) - 33.43 * Math.pow(distance, 4) + 2.593 * Math.pow(distance, 5);
        double A1 = 18.43145;
        double A2 = 67.62172;
        double x0 = 2.07751;
        double p = 5.16297;
        offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p));
        // debug("Shooter:", "offset = " + offset);
        // if (0 > offset || offset > 180) {
        //     debug("Shooter:", "wtf?");
        //     shooterSubsystem.getIo().setArmPosition(defaultAngle);
        //     return;
        // }

        if (Math.abs(
                offset -
                        Math.toDegrees(Units.rotationsToRadians(shooterSubsystem.armPosition.getValueAsDouble()))) >= 0.5) {
            defaultAngle = Degrees.of(offset);
            shooterSubsystem

                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(offset))
                    );
        }

		//System.out.println(swerve.getHeadingTarget());
		swerve.setHeadingTarget(swerve.getGyro().getYaw().getDegrees() - LimelightHelpers.getTX("limelight"));			
	}
	
	@Override
	public void end(boolean interrupted) {
		swerve.setLockHeading(false);
		shooterSubsystem.setArmPosition(Radians.zero());
		
	}
	
	
}