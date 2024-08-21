package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

public class IndexCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public IndexCommand(
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(indicatorSubsystem);
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.INDEXING);
    }

    @Override
    public void execute() {

        indexerSubsystem.getIo()
                .setIndexRPM(Constants.IndexerConstants.indexRPM);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo().setIndexRPM(0);
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.INDEX_FINISHING);
    }

    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getInputs().isIndexerBeamBreakOn;
    }
}