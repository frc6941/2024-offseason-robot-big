package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexOutCommand extends Command {
	private final IndexerSubsystem indexerSubsystem;

	public IndexOutCommand(IndexerSubsystem indexerSubsystem) {
		this.indexerSubsystem = indexerSubsystem;
		addRequirements(indexerSubsystem);
	}

	@Override
	public void execute() {
		indexerSubsystem.getIo()
				.setIndexRPM(-Constants.IndexerConstants.indexRPM);
	}

	@Override
	public void end(boolean interrupted) {
		indexerSubsystem.getIo().setIndexRPM(0);
	}
}
