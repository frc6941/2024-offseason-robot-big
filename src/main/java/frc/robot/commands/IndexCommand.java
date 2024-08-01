package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreakIORev;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intaker.IntakerIOTalonFX;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class IndexCommand extends Command {
    private final IndexerIOTalonFX indexerSubsystem;
    private final BeamBreakIORev beamBreakSubsystem;

    public IndexCommand(
            IndexerIOTalonFX indexerSubsystem,
            BeamBreakIORev beamBreakSubsystem
    ) {
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        indexerSubsystem
                .setIndexVoltage(Constants.IndexerConstants.indexVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.setIndexVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.indexerBeamBreak.get() &&
                !beamBreakSubsystem.intakerBeamBreak.get();
    }
}
