package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.indexShootVoltage;
import static frc.robot.Constants.IndexerConstants.trapIndexVoltage;

public class PreShootIndexCommand extends Command {
	private final IndexerSubsystem indexerSubsystem;
	private final ShooterSubsystem shooterSubsystem;
	private final Timer timer = new Timer();

	public PreShootIndexCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
		this.indexerSubsystem = indexerSubsystem;
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {
		timer.restart();
	}

	@Override
	public void execute() {
		indexerSubsystem.getIo().setIndexVoltage(trapIndexVoltage);
		shooterSubsystem.getIo().setShooterVoltage(trapIndexVoltage.mutableCopy().negate());
	}

	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.getIo().setShooterVoltage(Volts.zero());
		indexerSubsystem.getIo().setIndexVoltage(indexShootVoltage);
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(0.2);
	}
}
