package frc.robot.commands;

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

public class FerryShootCommand extends ParallelCommandGroup {
    public FerryShootCommand(
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Swerve swerve,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        addCommands(
                Commands.parallel(
                        new ChassisAimCommand(swerve, () -> Destination.FERRY, driverX, driverY).andThen(Commands.print("Spun Up")),
                        new ArmAimCommand(armSubsystem, () -> Destination.FERRY).andThen(Commands.print("Arm Aimed")),
                        new FlyWheelRampUp(shooterSubsystem, () -> Destination.FERRY).andThen(Commands.print("Spun Up")),
                        Commands.runOnce(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.FERRY_AIMING), indicatorSubsystem),
                        new WaitUntilCommand(() -> {
                            boolean swerveReady = swerve.aimingReady(10);
                            boolean shooterReady = shooterSubsystem.ShooterVelocityReady();
                            boolean armReady = armSubsystem.armAimingReady();
                            boolean swerveVelocityReady = swerve.velocityReady(1.3);
                            return swerveReady && shooterReady && armReady && swerveVelocityReady;
                            //&& !ShootingDecider.inHighFerryZone(swerve.getLocalizer().getCoarseFieldPose(0));
                        }).andThen(
                                Commands.waitSeconds(0.02),
                                new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem,
                                        indicatorSubsystem))));
    }
}
