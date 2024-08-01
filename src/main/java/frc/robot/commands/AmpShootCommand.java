package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.ShooterConstants.ampShootingVoltage;

public class AmpShootCommand extends ParallelCommandGroup {
	public AmpShootCommand(
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			BooleanSupplier confirmation) {
		addCommands(
				new AmpAimingCommand(shooterSubsystem),
				new PreShootWithoutAimingCommand(shooterSubsystem, ampShootingVoltage),
				Commands.sequence(
						new WaitUntilCommand(confirmation),
						new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
	}
}
