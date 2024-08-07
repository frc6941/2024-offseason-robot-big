package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PreShootWithoutAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double rpm;

    public PreShootWithoutAimingCommand(ShooterSubsystem shooterSubsystem, double rpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.rpm = rpm;

    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setFlyWheelVelocity(rpm);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
