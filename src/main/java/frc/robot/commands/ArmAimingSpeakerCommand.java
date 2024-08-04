package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ArmAimingSpeakerCommand extends Command {

	private ShooterSubsystem shooterSubsystem;
	
	public ArmAimingSpeakerCommand(
		ShooterSubsystem shooterSubsystem
	) {
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

	}

	@Override
	public void end() {

	}
}

//	while(		return (beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
// !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn);)
