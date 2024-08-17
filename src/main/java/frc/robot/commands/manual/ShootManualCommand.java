package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.shooting.ShootingDecider.Destination;

public class ShootManualCommand extends ParallelCommandGroup {
    public ShootManualCommand(
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IndicatorSubsystem indicatorSubsystem) {
        addCommands(
                new ArmAimManualCommand(armSubsystem, () -> Destination.SPEAKER),
                new FlyWheelManualRampUp(shooterSubsystem, () -> Destination.SPEAKER),
                Commands.runOnce(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.SPEAKER_AIMING), indicatorSubsystem)
        );
    }
}
