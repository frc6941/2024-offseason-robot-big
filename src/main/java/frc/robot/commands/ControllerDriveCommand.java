package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

public class ControllerDriveCommand extends Command {
	
	private final Supplier<Double> VelocityX, VelocityY, RotationalRate;
	private final Swerve swerve;

	public static double facingAngle;

	public ControllerDriveCommand(
			Swerve swerve,
			Supplier<Double> VelocityX,
			Supplier<Double> VelocityY,
			Supplier<Double> RotationalRate) {
		this.VelocityX = VelocityX;
		this.VelocityY = VelocityY;
		this.RotationalRate = RotationalRate;
		this.swerve = swerve;
		// System.out.println("VelocityX = " + this.VelocityX + "\n VelocityY = " + this.VelocityY + "\n RotationalRate = "
		// 		+ this.RotationalRate + "\n");
		//To prevent two commands which both requires swerveSubsystem from being scheduled simultaneously.
		addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
		swerve.setLockHeading(true);
		swerve.resetHeadingController();
		facingAngle = 0.0;
	}
	
	@Override
	public void execute() {
		swerve.drive(
				new Translation2d(
						VelocityX.get() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
						VelocityY.get() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
							 	0,
								true,
								false);
		facingAngle += Math.abs(RotationalRate.get()) 
				< Constants.SwerveDrivetrain.rotationalDeadband ? 
				0 : RotationalRate.get()
				* Math.toDegrees(Constants.SwerveDrivetrain.maxAngularRate.magnitude()) / 50;
		System.out.println(facingAngle);
		swerve.setHeadingTarget(facingAngle);					
	}
	
	@Override
	public void end(boolean interrupted) {
		
	}
	
	@Override
	public boolean isFinished() {
		return false;
	}
	
}