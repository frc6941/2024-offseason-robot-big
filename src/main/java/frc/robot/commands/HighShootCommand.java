package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.ShooterConstants.farShootVoltage;

public class HighShootCommand extends ParallelCommandGroup {
	public HighShootCommand(
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			Swerve Swerve,
			CommandXboxController drivercController) {
		addCommands(
				new ParallelAimingCommand(shooterSubsystem, Degrees.of(13)),
				new PreShootWithoutAimingCommand(shooterSubsystem, farShootVoltage),
				new LongShotCommand(shooterSubsystem, indicatorSubsystem, Swerve, drivercController),
				Commands.sequence(
						new WaitCommand(0.4),
						new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem))

		);
	}
}
