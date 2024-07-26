package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetFieldCentricCommand extends Command {
	SwerveSubsystem swerveSubsystem;
	boolean finished = false;

	public SetFieldCentricCommand(SwerveSubsystem swerveSubsystem) {
		this.swerveSubsystem = swerveSubsystem;
		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void execute() {
		swerveSubsystem.runOnce(swerveSubsystem::seedFieldRelative);
		finished = true;
		System.out.println("SeedFieldRelative Succeed");
	}
	
	@Override
	public void end(boolean interrupted) {
		System.out.println("SeedFieldRelative Ended");
	}
	@Override
	public boolean isFinished() {
		//System.out.println("SeedFieldRelative Finished");
		return finished;
	}
}
