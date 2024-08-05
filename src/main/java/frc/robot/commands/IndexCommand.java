package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class IndexCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private boolean enabledBefore = false;

    public IndexCommand(
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void initialize() {
        enabledBefore = false;
    }

    @Override
    public void execute() {
        if (isFinished()) {
            indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
            return;
        }
        if (beamBreakSubsystem.getInputs().isIntakerBeamBreakOn) {
            enabledBefore = true;
        }
        if (enabledBefore && !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn) {
            indexerSubsystem.getIo().setIndexVoltage(Volts.of(0.5));
            return;
        }
        indexerSubsystem.getIo()
                .setIndexVoltage(Constants.IndexerConstants.indexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getInputs().isIndexerBeamBreakOn;
    }
}
