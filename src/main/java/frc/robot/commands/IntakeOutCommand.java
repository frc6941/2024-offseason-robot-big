package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intaker.IntakerSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class IntakeOutCommand extends Command {
	private final IntakerSubsystem intakerSubsystem;

	public IntakeOutCommand(IntakerSubsystem intakerSubsystem) {
		this.intakerSubsystem = intakerSubsystem;
	}

	@Override
	public void execute() {
		intakerSubsystem.getIo()
				.setIntakeVoltage(Constants.IntakerConstants.intakeVoltage.mutableCopy().negate());
	}

	@Override
	public void end(boolean interrupted) {
		intakerSubsystem.getIo().setIntakeVoltage(Volts.zero());
	}
}
