package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;

public class SpeakerShootCommand extends ParallelCommandGroup {
	public SpeakerShootCommand(
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			Swerve Swerve,
			CommandXboxController driverController,
			BooleanSupplier confirmation) {
		addCommands(
				new SpeakerAimingCommand(shooterSubsystem, indicatorSubsystem, Swerve, driverController),
				new PreShootCommand(shooterSubsystem),
				Commands.sequence(
						new WaitUntilCommand(confirmation),
						new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
	}
}