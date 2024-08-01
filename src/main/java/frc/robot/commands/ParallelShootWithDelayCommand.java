package frc.robot.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.farShootVoltage;

public class ParallelShootWithDelayCommand extends ParallelCommandGroup {
	public ParallelShootWithDelayCommand(
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			Measure<Angle> angle) {
		addCommands(
				new ParallelAimingCommand(shooterSubsystem, angle),
				new PreShootWithoutAimingCommand(shooterSubsystem, farShootVoltage),
				Commands.sequence(
						new WaitCommand(0.5),
						new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
	}
}