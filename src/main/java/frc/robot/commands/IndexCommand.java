package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;

    public IndexCommand(
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (isFinished()) {
            indexerSubsystem.getIo().setIndexRPM(0);
            return;
        }
        indexerSubsystem.getIo()
                .setIndexRPM(Constants.IndexerConstants.indexRPM);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo().setIndexRPM(0);
    }

    @Override
    public boolean isFinished() {
        return RobotBase.isSimulation() || beamBreakSubsystem.getInputs().isIndexerBeamBreakOn;
    }
}
