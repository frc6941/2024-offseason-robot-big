package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Logger.debug;

public class IntakeCommand extends Command {
	private final IntakerSubsystem intakerSubsystem;
	private final BeamBreakSubsystem beamBreakSubsystem;
	private final IndicatorSubsystem indicatorSubsystem;
	private final ShooterSubsystem shooterSubsystem;

	public IntakeCommand(
			IntakerSubsystem intakerSubsystem,
			BeamBreakSubsystem beamBreakSubsystem,
			IndicatorSubsystem indicatorSubsystem,
			ShooterSubsystem shooterSubsystem) {
		this.intakerSubsystem = intakerSubsystem;
		this.beamBreakSubsystem = beamBreakSubsystem;
		this.indicatorSubsystem = indicatorSubsystem;
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void execute() {
		if (isFinished())
			return;
		if ((shooterSubsystem.getInputs().armPosition.magnitude() > 0.1
				&& beamBreakSubsystem.getInputs().isIntakerBeamBreakOn)) {
			intakerSubsystem.getIo().setIntakeVoltage(Volts.of(0));
		} else {
			intakerSubsystem.getIo().setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
		}

		shooterSubsystem.getIo().setShooterVoltage(Volts.of(1));

		// debug(" " + shooterSubsystem.getInputs().armPosition.magnitude());
	}

	@Override
	public void end(boolean interrupted) {
		intakerSubsystem.getIo()
				.setIntakeVoltage(Volts.zero());
		shooterSubsystem.getIo()
				.setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
		if (interrupted)
			return;
		indicatorSubsystem
				.setPattern(IndicatorIO.Patterns.FINISH_INTAKE);
	}

	@Override
	public boolean isFinished() {
		return (beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
				!beamBreakSubsystem.getInputs().isIntakerBeamBreakOn);
	}
}
