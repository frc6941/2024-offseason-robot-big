package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Seconds;

public class StartClimbCommand extends ParallelCommandGroup {
    public StartClimbCommand(
            ArmSubsystem armSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            CommandXboxController driverController,
            CommandXboxController operatorController,
            BooleanSupplier confirmation
    ) {
        addCommands(
                new ClimbShooterUpCommand(armSubsystem),
                new SequentialCommandGroup(
                        Commands.runOnce(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.CAN_CLIMB)),
                        new RumbleCommand(Seconds.of(0.5), driverController.getHID(), operatorController.getHID()),
                        new WaitUntilCommand(confirmation),
                        new ClimbEndgameCommand(armSubsystem, indicatorSubsystem)
                )
        );
    }
}
