package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.shooterIndexVoltage;

public class DeliverNoteIndexCommand extends Command {
	private final ShooterSubsystem shooterSubsystem;
	private final Timer timer = new Timer();
	private final IndexerSubsystem indexerSubsystem;

	public DeliverNoteIndexCommand(
			ShooterSubsystem shooterSubsystem,
			IndexerSubsystem indexerSubsystem) {
		this.shooterSubsystem = shooterSubsystem;
		this.indexerSubsystem = indexerSubsystem;
	}

	@Override
	public void initialize() {
		shooterSubsystem.getIo().setFlyWheelVoltage(Volts.zero());
		timer.restart();
	}

	@Override
	public void execute() {
		// Yes, it's weird. Not my idea.
		shooterSubsystem.getIo().setFlyWheelVoltage(shooterIndexVoltage);
	}

	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.getIo().setFlyWheelVoltage(Constants.ShooterConstants.shooterConstantVoltage);
		indexerSubsystem.getIo().setIndexVoltage(Volts.zero());
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(1.0);
	}
}
