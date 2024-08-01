package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Radians;

public class ShootPlateCommand extends ParallelCommandGroup {
	public ShootPlateCommand(
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			BooleanSupplier confirmation) {
		addCommands(
				new PreShootIndexCommand(indexerSubsystem, shooterSubsystem),
				Commands.sequence(
						new ShooterUpCommand(shooterSubsystem).onlyWhile(
								// magic number; do not touch!
								() -> shooterSubsystem.getInputs().armPosition.lt(Radians.of(3.767))),
						new WaitUntilCommand(confirmation),
						new DeliverNoteIndexCommand(shooterSubsystem, indexerSubsystem)));
	}
}
