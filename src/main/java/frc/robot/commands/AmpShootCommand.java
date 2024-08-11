package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.shooting.ShootingDecider.Destination;

import java.util.function.BooleanSupplier;

public class AmpShootCommand extends ParallelCommandGroup {
    public AmpShootCommand(
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            BooleanSupplier confirmation) {
        addCommands(
                new ArmAimCommand(armSubsystem, () -> Destination.AMP),
                new FlyWheelRampUp(shooterSubsystem, () -> Destination.AMP),
                Commands.sequence(
                        new WaitUntilCommand(confirmation),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
    }
}
