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
import frc.robot.utils.shooting.ShootingDecider.Destination;

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
                Commands.parallel(
                        new ChassisAimCommand(Swerve, () -> Destination.FERRY, driverX, driverY).andThen(Commands.print("Spun Up")),
                        new ArmAimCommand(shooterSubsystem, () -> Destination.FERRY).andThen(Commands.print("Arm Aimed")),
                        new FlyWheelRampUp(shooterSubsystem, () -> Destination.FERRY).andThen(Commands.print("Spun Up")),
                        new WaitUntilCommand(() -> (Swerve.aimingReady(2.5) &&
                                shooterSubsystem.aimingReady())).andThen(
                                        Commands.waitSeconds(0.02),
                                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem,
                                                indicatorSubsystem))));
    }
}