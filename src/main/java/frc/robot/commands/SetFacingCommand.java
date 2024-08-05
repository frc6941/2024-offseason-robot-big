package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class SetFacingCommand extends Command {
	Swerve swerve;
	double facingAngle;
	double startTime;
	double timeout = 1;
	int finishedCnt = 0;
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
		startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		swerve.setHeadingTarget(facingAngle);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.setLockHeading(false);
	}

	@Override
	public boolean isFinished() {
		if (swerve.aimingReady(1)) {
			finishedCnt++;
			if (finishedCnt == 10)
				return true;
		} else {
			finishedCnt = 0;
		}
		return false || Timer.getFPGATimestamp() - startTime > timeout;
	}

}
