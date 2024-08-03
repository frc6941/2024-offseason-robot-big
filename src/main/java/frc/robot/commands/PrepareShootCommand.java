package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class PrepareShootCommand extends Command {
	private final ShooterSubsystem shooterSubsystem;

	public PrepareShootCommand(ShooterSubsystem shooterSubsystem) {
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		shooterSubsystem.getIo().setFlyWheelVoltage(Volts.of(-8));
	}

	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.getIo()
				.setFlyWheelVoltage(Constants.ShooterConstants.shooterConstantVoltage);
	}
}
