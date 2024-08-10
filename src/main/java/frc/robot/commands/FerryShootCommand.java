package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

public class FerryShootCommand extends ParallelCommandGroup {
    public FerryShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Swerve Swerve,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        addCommands(
                // new SpeakerAimingCommand(shooterSubsystem, indicatorSubsystem, beamBreakSubsystem, Swerve, driverX, driverY),
                // new PreShootCommand(shooterSubsystem),
                Commands.sequence(
                        new ArmAimCommand(shooterSubsystem, null),
                        new FlyWheelRampUp(shooterSubsystem, null),        
                        new WaitUntilCommand(() -> (
                                Swerve.aimingReady(2.5) &&
                                        shooterSubsystem.aimingReady()
                        )),
                        Commands.runOnce(() -> Timer.delay(0.02), indicatorSubsystem),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
    }
}
