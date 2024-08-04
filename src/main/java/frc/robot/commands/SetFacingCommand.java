package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class SetFacingCommand extends Command {
	Swerve swerve;
	double facingAngle;
	public SetFacingCommand(
		Swerve swerve,
		double facingAngle
	) {

		this.swerve = swerve;
		this.facingAngle = facingAngle;
	}
	
	@Override
	public void initialize() {
		swerve.setLockHeading(true);
	}

	@Override
	public void execute() {
		swerve.setHeadingTarget(facingAngle);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.setLockHeading(false);
	}

}
