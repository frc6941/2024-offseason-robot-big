package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Degrees;

public class ShooterUpCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterUpCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(true);
    }

    @Override
    public void execute() {
        Constants.armPosition = Constants.armPosition.plus(Degrees.of(1));
        shooterSubsystem.getIo()
                .setArmPosition(Constants.armPosition);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setPullerBrakeMode(false);
    }
}
