package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ArmAimCommand;
import frc.robot.commands.DeliverNoteCommand;
import frc.robot.commands.FlyWheelRampUp;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider.Destination;

public class SpeakerShootAutoCommand extends ParallelCommandGroup {
    public SpeakerShootAutoCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Swerve Swerve) {
        addCommands(
                new ChassisAimAutoCommand(Swerve, () -> Destination.SPEAKER),
                new ArmAimCommand(shooterSubsystem, () -> Destination.SPEAKER),
                new FlyWheelRampUp(shooterSubsystem, () -> Destination.SPEAKER),
                Commands.sequence(
                        new WaitUntilCommand(() -> shooterSubsystem.aimingReady()),
                        Commands.runOnce(() -> Timer.delay(0.02), indicatorSubsystem),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
    }
}
