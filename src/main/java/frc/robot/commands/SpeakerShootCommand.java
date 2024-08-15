package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider.Destination;

import java.util.function.DoubleSupplier;

public class SpeakerShootCommand extends ParallelCommandGroup {
    public SpeakerShootCommand(
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Swerve Swerve,
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            boolean isAuto) {
        addCommands(
                Commands.deadline(
                        Commands.sequence(
                                new WaitUntilCommand(() -> {
                                    boolean swerveReady = Swerve.aimingReady(2.5);
                                    boolean shooterReady = shooterSubsystem.ShooterVelocityReady();
                                    boolean armReady = armSubsystem.armAimingReady();
                                    return swerveReady && shooterReady && armReady;
                                }),
                                Commands.runOnce(() -> Timer.delay(0.02)),
                                new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)),
                        new ChassisAimCommand(Swerve, () -> Destination.SPEAKER, driverX, driverY, isAuto),
                        new ArmAimCommand(armSubsystem, () -> Destination.SPEAKER),
                        new FlyWheelRampUp(shooterSubsystem, () -> Destination.SPEAKER),
                        Commands.runOnce(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.SPEAKER_AIMING), indicatorSubsystem)
                ));

    }

    public SpeakerShootCommand(
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Swerve Swerve) {
        this(shooterSubsystem, armSubsystem, indexerSubsystem, beamBreakSubsystem, indicatorSubsystem, Swerve, () -> 0, () -> 0, true);
    }
}
