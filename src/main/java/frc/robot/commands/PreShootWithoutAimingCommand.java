package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PreShootWithoutAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Measure<Voltage> volts;

    public PreShootWithoutAimingCommand(ShooterSubsystem shooterSubsystem, Measure<Voltage> volts) {
        this.shooterSubsystem = shooterSubsystem;
        this.volts = volts;

    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setFlyWheelVoltage(volts);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
