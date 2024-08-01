package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.ShooterConstants.farShootVoltage;

public class ShootWithoutAimingCommand extends ParallelCommandGroup {
	public ShootWithoutAimingCommand(
			IndicatorSubsystem indicatorSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem,
			Supplier<Boolean> confirmation) {
		addCommands(
				new PreShootWithoutAimingCommand(shooterSubsystem, farShootVoltage),
				Commands.sequence(
						new WaitUntilCommand(confirmation::get),
						new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
	}
}
