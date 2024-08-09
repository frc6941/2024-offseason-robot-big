package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.utils.TunableNumber;

import static frc.robot.Constants.IndexerConstants.indexRPM;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class IndexTestCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private TunableNumber IndexerRPM = new TunableNumber("IndexerRPM", 0.0);

    public IndexTestCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        indexerSubsystem.getIo().setIndexRPM(IndexerRPM.get());
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.getIo()
                .setIndexRPM(0);
    }
}
