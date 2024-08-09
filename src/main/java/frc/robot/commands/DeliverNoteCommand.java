package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.indexRPM;
import static frc.robot.Constants.IndexerConstants.indexVoltage;
import static frc.robot.Constants.IndexerConstants.triggerRPM;

public class DeliverNoteCommand extends Command {
	private final IndexerSubsystem indexerSubsystem;
	private final BeamBreakSubsystem beamBreakSubsystem;
	private final IndicatorSubsystem indicatorSubsystem;

	public DeliverNoteCommand(
			IndexerSubsystem indexerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			IndicatorSubsystem indicatorSubsystem) {
		this.indexerSubsystem = indexerSubsystem;
		this.beamBreakSubsystem = beamBreakSubsystem;
		this.indicatorSubsystem = indicatorSubsystem;
	}

	@Override
	public void execute() {
		indexerSubsystem.getIo().setIndexRPM(triggerRPM);
	}

	@Override
	public void end(boolean interrupted) {
		indexerSubsystem.getIo().setIndexRPM(0);
		if (interrupted)
			return;
		indicatorSubsystem
				.setPattern(IndicatorIO.Patterns.FINISH_SHOOT);
	}

	@Override
	public boolean isFinished() {
		return !beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
				!beamBreakSubsystem.getInputs().isShooterBeamBreakOn;
	}
}
